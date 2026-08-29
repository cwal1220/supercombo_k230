#include "recording_writer.h"

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/statvfs.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

namespace {

constexpr uint64_t kSegmentDurationNs = 60ULL * 1000000000ULL;
constexpr uint64_t kStorageCheckIntervalNs = 5ULL * 1000000000ULL;
constexpr uint64_t kMinimumFreeBytes = 5ULL * 1024 * 1024 * 1024;
constexpr unsigned kMinimumFreePercent = 10;
constexpr size_t kFileBufferBytes = 1024 * 1024;

bool make_directories(const std::string &path) {
  if (path.empty()) return false;
  std::string current;
  if (path.front() == '/') current = "/";
  size_t start = path.front() == '/' ? 1 : 0;
  while (start <= path.size()) {
    const size_t slash = path.find('/', start);
    const std::string component = path.substr(start, slash - start);
    if (!component.empty()) {
      if (!current.empty() && current.back() != '/') current += '/';
      current += component;
      if (mkdir(current.c_str(), 0775) != 0 && errno != EEXIST) return false;
    }
    if (slash == std::string::npos) break;
    start = slash + 1;
  }
  return true;
}

/* 보드에는 tz 데이터가 없어 localtime이 UTC로 떨어진다. route 이름은 주행을
 * 되짚는 사람이 읽는 값이므로 KST(DST 없음)로 고정한다. */
constexpr std::chrono::hours kKoreaStandardTimeOffset{9};

std::string route_name() {
  const auto now = std::chrono::system_clock::now();
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
  const std::time_t wall =
      std::chrono::system_clock::to_time_t(now + kKoreaStandardTimeOffset);
  std::tm korea{};
  gmtime_r(&wall, &korea);
  std::ostringstream name;
  name << std::put_time(&korea, "%Y-%m-%d--%H-%M-%S") << '-'
       << std::setw(3) << std::setfill('0') << milliseconds.count();
  return name.str();
}

bool copy_file(const std::string &source, const std::string &destination) {
  std::ifstream input(source, std::ios::binary);
  if (!input) return false;
  std::ofstream output(destination, std::ios::binary | std::ios::trunc);
  if (!output) return false;
  output << input.rdbuf();
  return (input.good() || input.eof()) && output.good();
}

}  // namespace

RecordingWriter::RecordingWriter(std::string root, std::string params_directory,
                                 unsigned width, unsigned height, unsigned fps,
                                 unsigned bitrate)
    : root_(std::move(root)), params_directory_(std::move(params_directory)),
      width_(width), height_(height), fps_(fps), bitrate_(bitrate),
      worker_(&RecordingWriter::worker_loop, this) {
  const char *staging = std::getenv("K230_RECORD_STAGING");
  staging_root_ = staging && staging[0] != '\0' ? staging : "/tmp/record_staging";
  mover_ = std::thread(&RecordingWriter::mover_loop, this);
  /* 이전 세션이 route 도중 죽었으면 스테이징 잔여가 tmpfs(램)를 계속
   * 점유한다. 시작할 때 남아 있는 route를 SD로 회수한다. */
  if (DIR *stale = opendir(staging_root_.c_str())) {
    while (dirent *entry = readdir(stale)) {
      const std::string name = entry->d_name;
      if (name == "." || name == "..") continue;
      MoveJob job;
      job.tree = true;
      job.from = staging_root_ + "/" + name;
      job.to = root_ + "/" + name;
      std::fprintf(stderr, "recordd: recovering staged route %s\n", name.c_str());
      enqueue_move(std::move(job));
    }
    closedir(stale);
  }
}

RecordingWriter::~RecordingWriter() {
  close();
}

void RecordingWriter::enqueue(PendingWrite &&write, bool force) {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!force && queue_.size() >= kMaximumPendingWrites) {
      queue_drops_.fetch_add(1);
      return;
    }
    queue_.push_back(std::move(write));
  }
  queue_cv_.notify_one();
}

void RecordingWriter::worker_loop() {
  while (true) {
    PendingWrite write;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return !queue_.empty(); });
      write = std::move(queue_.front());
      queue_.pop_front();
    }
    const bool stop = write.kind == PendingWrite::Kind::Stop;
    process(std::move(write));
    if (stop) return;
  }
}

void RecordingWriter::process(PendingWrite &&write) {
  switch (write.kind) {
    case PendingWrite::Kind::Enable:
      if (!start_route(write.timestamp_ns)) {
        requested_enabled_.store(false);
      }
      break;
    case PendingWrite::Kind::Disable:
      close_route(true);
      break;
    case PendingWrite::Kind::CodecConfig:
      codec_config_ = std::move(write.data);
      break;
    case PendingWrite::Kind::EncodedFrame:
      write_encoded_frame_impl(write.frame, write.data.data(), write.data.size(),
                               write.keyframe);
      break;
    case PendingWrite::Kind::Can:
      write_can_impl(write.record_type, write.can_batch);
      break;
    case PendingWrite::Kind::State:
      write_state_impl(write.record_type, write.timestamp_ns, write.data.data(),
                       write.data.size());
      break;
    case PendingWrite::Kind::Stop:
      close_route(true);
      break;
  }
}

FILE *RecordingWriter::open_buffered(const std::string &path) {
  FILE *file = std::fopen(path.c_str(), "wb");
  if (file) setvbuf(file, nullptr, _IOFBF, kFileBufferBytes);
  return file;
}

bool RecordingWriter::has_storage_reserve() const {
  struct statvfs space {};
  if (statvfs(root_.c_str(), &space) != 0 || space.f_blocks == 0) return false;
  const uint64_t block_size = space.f_frsize ? space.f_frsize : space.f_bsize;
  const uint64_t free_bytes = block_size * space.f_bavail;
  const uint64_t total_bytes = block_size * space.f_blocks;
  return free_bytes >= kMinimumFreeBytes &&
      free_bytes * 100ULL >= total_bytes * kMinimumFreePercent;
}

void RecordingWriter::set_enabled(bool enabled, uint64_t now_ns) {
  if (!enabled) {
    if (!requested_enabled_.exchange(false)) return;
    blocked_for_space_.store(false);
    PendingWrite write;
    write.kind = PendingWrite::Kind::Disable;
    enqueue(std::move(write), true);
    return;
  }
  if (requested_enabled_.exchange(true)) return;
  blocked_for_space_.store(false);
  PendingWrite write;
  write.kind = PendingWrite::Kind::Enable;
  write.timestamp_ns = now_ns;
  enqueue(std::move(write), true);
}

bool RecordingWriter::start_route(uint64_t now_ns) {
  if (!make_directories(root_) || !has_storage_reserve()) {
    std::fprintf(stderr, "recordd: recording refused: storage reserve is below 5 GiB/10%%\n");
    blocked_for_space_.store(true);
    active_.store(false);
    return false;
  }
  /* 활성 route는 tmpfs에 쓰고 mover가 닫힌 파일을 SD로 옮긴다. */
  const std::string name = route_name();
  route_path_ = staging_root_ + "/" + name;
  final_route_path_ = root_ + "/" + name;
  if (!make_directories(route_path_ + "/segments")) {
    std::fprintf(stderr, "recordd: create route failed path=%s error=%s\n",
                 route_path_.c_str(), std::strerror(errno));
    return false;
  }
  segment_index_ = 0;
  total_video_frames_.store(0);
  event_records_ = 0;
  route_start_ns_ = now_ns;
  event_chunk_index_ = 0;
  /* 이벤트 로그는 route 단위 단일 파일이 아니라 60초 청크다. CAN 로깅만으로
   * ~0.5 MB/s가 쌓이므로 route 단위 파일은 긴 주행에서 tmpfs 스테이징을
   * 가득 채운다(988 MB / 30분). 닫힌 청크는 세그먼트처럼 mover가 옮긴다. */
  if (!make_directories(route_path_ + "/events") || !open_event_chunk(now_ns)) {
    std::fprintf(stderr, "recordd: open event log failed: %s\n", std::strerror(errno));
    close_route(false);
    return false;
  }
  snapshot_params();
  write_manifest(false);
  next_storage_check_ns_ = now_ns + kStorageCheckIntervalNs;
  active_.store(true);
  std::fprintf(stderr, "recordd: recording started route=%s staging=%s\n",
               final_route_path_.c_str(), route_path_.c_str());
  return true;
}

void RecordingWriter::set_codec_config(const uint8_t *data, size_t size) {
  if (!data || size == 0) return;
  PendingWrite write;
  write.kind = PendingWrite::Kind::CodecConfig;
  write.data.assign(data, data + size);
  enqueue(std::move(write));
}

bool RecordingWriter::open_segment(const K230RoadAiFrame &frame) {
  std::ostringstream number;
  number << std::setw(3) << std::setfill('0') << segment_index_;
  segment_relative_ = "segments/" + number.str();
  const std::string directory = route_path_ + "/" + segment_relative_;
  if (!make_directories(directory)) return false;
  video_file_ = open_buffered(directory + "/road.hevc");
  index_file_ = open_buffered(directory + "/frames.bin");
  if (!video_file_ || !index_file_) {
    close_segment();
    return false;
  }
  segment_start_ns_ = frame.timestamp_ns;
  video_offset_ = 0;
  if (!codec_config_.empty()) {
    std::fwrite(codec_config_.data(), 1, codec_config_.size(), video_file_);
    video_offset_ += codec_config_.size();
  }
  K230FrameIndexHeader header;
  header.record_size = sizeof(K230FrameIndexRecord);
  header.width = width_;
  header.height = height_;
  header.fps = fps_;
  header.segment_start_ns = frame.timestamp_ns;
  if (std::fwrite(&header, sizeof(header), 1, index_file_) != 1) {
    close_segment();
    return false;
  }
  return true;
}

void RecordingWriter::write_encoded_frame(const K230RoadAiFrame &frame,
                                           const uint8_t *data, size_t size,
                                           bool keyframe) {
  if (!requested_enabled_.load() || !data || size == 0) return;
  PendingWrite write;
  write.kind = PendingWrite::Kind::EncodedFrame;
  write.frame = frame;
  write.keyframe = keyframe;
  write.data.assign(data, data + size);
  enqueue(std::move(write));
}

void RecordingWriter::write_encoded_frame_impl(const K230RoadAiFrame &frame,
                                                const uint8_t *data, size_t size,
                                                bool keyframe) {
  if (!event_file_) return;
  if (frame.timestamp_ns >= next_storage_check_ns_) {
    next_storage_check_ns_ = frame.timestamp_ns + kStorageCheckIntervalNs;
    if (!has_storage_reserve()) {
      blocked_for_space_.store(true);
      requested_enabled_.store(false);
      close_route(true);
      std::fprintf(stderr, "recordd: recording stopped to preserve storage reserve\n");
      return;
    }
  }
  if (!video_file_) {
    if (!keyframe || codec_config_.empty() || !open_segment(frame)) return;
  } else if (keyframe && frame.timestamp_ns - segment_start_ns_ >= kSegmentDurationNs) {
    close_segment();
    ++segment_index_;
    if (!open_segment(frame)) return;
  }

  const uint64_t offset = video_offset_;
  if (std::fwrite(data, 1, size, video_file_) != size) {
    std::fprintf(stderr, "recordd: video write failed: %s\n", std::strerror(errno));
    return;
  }
  K230FrameIndexRecord index;
  index.frame_id = frame.frame_id;
  index.capture_timestamp_ns = frame.timestamp_ns;
  index.encode_index = total_video_frames_.load();
  index.file_offset = offset;
  index.packet_size = static_cast<uint32_t>(size);
  index.flags = keyframe ? 1U : 0U;
  if (std::fwrite(&index, sizeof(index), 1, index_file_) != 1) {
    std::fprintf(stderr, "recordd: frame index write failed: %s\n", std::strerror(errno));
    return;
  }
  video_offset_ += size;
  total_video_frames_.fetch_add(1);
}

bool RecordingWriter::open_event_chunk(uint64_t now_ns) {
  std::ostringstream number;
  number << std::setw(3) << std::setfill('0') << event_chunk_index_;
  event_file_ = open_buffered(route_path_ + "/events/" + number.str() + ".bin");
  if (!event_file_) return false;
  event_chunk_start_ns_ = now_ns;
  K230EventFileHeader header;
  header.route_start_ns = route_start_ns_;
  if (std::fwrite(&header, sizeof(header), 1, event_file_) != 1) {
    std::fclose(event_file_);
    event_file_ = nullptr;
    return false;
  }
  return true;
}

void RecordingWriter::close_event_chunk() {
  if (!event_file_) return;
  std::fclose(event_file_);
  event_file_ = nullptr;
  std::ostringstream number;
  number << std::setw(3) << std::setfill('0') << event_chunk_index_;
  MoveJob job;
  job.from = route_path_ + "/events/" + number.str() + ".bin";
  job.to = final_route_path_ + "/events/" + number.str() + ".bin";
  enqueue_move(std::move(job));
}

bool RecordingWriter::write_event_header(K230RecordType type, uint64_t timestamp_ns,
                                          uint32_t payload_size) {
  if (!event_file_) return false;
  if (timestamp_ns >= event_chunk_start_ns_ &&
      timestamp_ns - event_chunk_start_ns_ >= kSegmentDurationNs) {
    close_event_chunk();
    ++event_chunk_index_;
    if (!open_event_chunk(timestamp_ns)) {
      std::fprintf(stderr, "recordd: open event chunk failed: %s\n",
                   std::strerror(errno));
      return false;
    }
  }
  K230EventRecordHeader header;
  header.timestamp_ns = timestamp_ns;
  header.type = static_cast<uint16_t>(type);
  header.payload_size = payload_size;
  return std::fwrite(&header, sizeof(header), 1, event_file_) == 1;
}

void RecordingWriter::write_can(K230RecordType type, const K230CanBatch &batch) {
  if (!requested_enabled_.load() ||
      (type != K230RecordType::CanRx && type != K230RecordType::CanTx)) return;
  PendingWrite write;
  write.kind = PendingWrite::Kind::Can;
  write.record_type = type;
  write.can_batch = batch;
  enqueue(std::move(write));
}

void RecordingWriter::write_can_impl(K230RecordType type,
                                     const K230CanBatch &batch) {
  if (!event_file_) return;
  const uint32_t count = std::min<uint32_t>(batch.count, kK230CanBatchMaxFrames);
  const uint32_t payload_size = sizeof(K230RecordedCanBatchHeader) +
      count * sizeof(K230RecordedCanFrame);
  if (!write_event_header(type, batch.timestamp_ns, payload_size)) return;
  K230RecordedCanBatchHeader batch_header{count, batch.dropped};
  std::fwrite(&batch_header, sizeof(batch_header), 1, event_file_);
  for (uint32_t index = 0; index < count; ++index) {
    const K230CanFrame &source = batch.frames[index];
    K230RecordedCanFrame recorded;
    recorded.address = source.address;
    recorded.src = source.src;
    recorded.bus_time = source.bus_time;
    recorded.data_len = source.data_len;
    recorded.flags = source.flags;
    std::memcpy(recorded.data, source.data, sizeof(recorded.data));
    std::fwrite(&recorded, sizeof(recorded), 1, event_file_);
  }
  ++event_records_;
}

void RecordingWriter::write_state(K230RecordType type, uint64_t timestamp_ns,
                                  const void *data, size_t size) {
  if (!requested_enabled_.load() || !data ||
      size == 0 || size > UINT32_MAX) return;
  PendingWrite write;
  write.kind = PendingWrite::Kind::State;
  write.record_type = type;
  write.timestamp_ns = timestamp_ns;
  write.data.assign(static_cast<const uint8_t *>(data),
                    static_cast<const uint8_t *>(data) + size);
  enqueue(std::move(write));
}

void RecordingWriter::write_state_impl(K230RecordType type, uint64_t timestamp_ns,
                                       const void *data, size_t size) {
  if (!event_file_) return;
  if (!write_event_header(type, timestamp_ns, static_cast<uint32_t>(size))) return;
  if (std::fwrite(data, 1, size, event_file_) == size) ++event_records_;
}

void RecordingWriter::close_segment() {
  const bool had_files = video_file_ != nullptr || index_file_ != nullptr;
  if (video_file_) std::fclose(video_file_);
  if (index_file_) std::fclose(index_file_);
  video_file_ = nullptr;
  index_file_ = nullptr;
  segment_start_ns_ = 0;
  video_offset_ = 0;
  if (had_files && !segment_relative_.empty()) {
    for (const char *file : {"/road.hevc", "/frames.bin"}) {
      MoveJob job;
      job.from = route_path_ + "/" + segment_relative_ + file;
      job.to = final_route_path_ + "/" + segment_relative_ + file;
      enqueue_move(std::move(job));
    }
  }
  segment_relative_.clear();
}

void RecordingWriter::write_manifest(bool complete) const {
  if (route_path_.empty()) return;
  std::ofstream manifest(route_path_ + "/manifest.json", std::ios::trunc);
  manifest << "{\n"
           << "  \"version\": " << kK230RecordingVersion << ",\n"
           << "  \"complete\": " << (complete ? "true" : "false") << ",\n"
           << "  \"video_codec\": \"hevc\",\n"
           << "  \"width\": " << width_ << ",\n"
           << "  \"height\": " << height_ << ",\n"
           << "  \"fps\": " << fps_ << ",\n"
           << "  \"bitrate\": " << bitrate_ << ",\n"
           << "  \"segment_seconds\": 60,\n"
           << "  \"video_frames\": " << total_video_frames_.load() << ",\n"
           << "  \"event_records\": " << event_records_ << "\n"
           << "}\n";
}

void RecordingWriter::snapshot_params() const {
  DIR *directory = opendir(params_directory_.c_str());
  if (!directory) return;
  const std::string destination = route_path_ + "/params";
  make_directories(destination);
  while (dirent *entry = readdir(directory)) {
    const std::string name = entry->d_name;
    if (name.size() < 5 || name.substr(name.size() - 5) != ".json") continue;
    copy_file(params_directory_ + "/" + name, destination + "/" + name);
  }
  closedir(directory);
}

void RecordingWriter::close_route(bool complete) {
  if (!event_file_ && route_path_.empty()) return;
  close_segment();
  close_event_chunk();
  write_manifest(complete);
  if (!route_path_.empty()) {
    std::fprintf(stderr,
                 "recordd: recording stopped route=%s frames=%llu events=%llu\n",
                 final_route_path_.c_str(),
                 static_cast<unsigned long long>(total_video_frames_.load()),
                 static_cast<unsigned long long>(event_records_));
    /* 남은 route 파일(events.bin, manifest, params, 마지막 세그먼트)을
     * 전부 SD로 옮긴다. mover는 순서대로 처리하므로 앞선 파일 이동이
     * 끝난 뒤 잔여만 쓸어 담는다. */
    MoveJob sweep;
    sweep.tree = true;
    sweep.from = route_path_;
    sweep.to = final_route_path_;
    enqueue_move(std::move(sweep));
  }
  route_path_.clear();
  final_route_path_.clear();
  segment_index_ = 0;
  active_.store(false);
}

void RecordingWriter::close() {
  if (!worker_.joinable()) return;
  requested_enabled_.store(false);
  PendingWrite write;
  write.kind = PendingWrite::Kind::Stop;
  enqueue(std::move(write), true);
  if (worker_.joinable()) worker_.join();
  {
    std::lock_guard<std::mutex> lock(move_mutex_);
    mover_stop_ = true;
  }
  move_cv_.notify_one();
  if (mover_.joinable()) mover_.join();
}

void RecordingWriter::enqueue_move(MoveJob &&job) {
  {
    std::lock_guard<std::mutex> lock(move_mutex_);
    move_queue_.push_back(std::move(job));
    pending_moves_.store(move_queue_.size());
  }
  move_cv_.notify_one();
}

void RecordingWriter::mover_loop() {
  while (true) {
    MoveJob job;
    {
      std::unique_lock<std::mutex> lock(move_mutex_);
      move_cv_.wait(lock, [this] { return mover_stop_ || !move_queue_.empty(); });
      if (move_queue_.empty()) {
        if (mover_stop_) return;
        continue;
      }
      job = std::move(move_queue_.front());
      move_queue_.pop_front();
      pending_moves_.store(move_queue_.size());
    }
    if (job.tree) {
      move_tree(job.from, job.to);
    } else {
      move_file(job.from, job.to);
    }
  }
}

void RecordingWriter::move_file(const std::string &from, const std::string &to) {
  const size_t slash = to.rfind('/');
  if (slash != std::string::npos) make_directories(to.substr(0, slash));
  if (!copy_file(from, to)) {
    std::fprintf(stderr, "recordd: move failed %s -> %s: %s\n",
                 from.c_str(), to.c_str(), std::strerror(errno));
    return;
  }
  unlink(from.c_str());
}

void RecordingWriter::move_tree(const std::string &from, const std::string &to) {
  DIR *directory = opendir(from.c_str());
  if (!directory) return;
  while (dirent *entry = readdir(directory)) {
    const std::string name = entry->d_name;
    if (name == "." || name == "..") continue;
    const std::string source = from + "/" + name;
    struct stat info = {};
    if (stat(source.c_str(), &info) != 0) continue;
    if (S_ISDIR(info.st_mode)) {
      move_tree(source, to + "/" + name);
    } else {
      move_file(source, to + "/" + name);
    }
  }
  closedir(directory);
  rmdir(from.c_str());
}
