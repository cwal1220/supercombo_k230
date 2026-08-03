#include "recording_writer.h"

#include <dirent.h>
#include <sys/stat.h>
#include <sys/statvfs.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <ctime>
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

std::string route_name() {
  const auto now = std::chrono::system_clock::now();
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
  const std::time_t wall = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&wall, &local);
  std::ostringstream name;
  name << std::put_time(&local, "%Y-%m-%d--%H-%M-%S") << '-'
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
      width_(width), height_(height), fps_(fps), bitrate_(bitrate) {}

RecordingWriter::~RecordingWriter() {
  close();
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
    requested_enabled_ = false;
    if (active()) close_route(true);
    blocked_for_space_ = false;
    return;
  }
  if (requested_enabled_) return;
  requested_enabled_ = true;
  blocked_for_space_ = false;
  if (!start_route(now_ns)) requested_enabled_ = false;
}

bool RecordingWriter::start_route(uint64_t now_ns) {
  if (!make_directories(root_) || !has_storage_reserve()) {
    std::fprintf(stderr, "recordd: recording refused: storage reserve is below 5 GiB/10%%\n");
    blocked_for_space_ = true;
    return false;
  }
  route_path_ = root_ + "/" + route_name();
  if (!make_directories(route_path_ + "/segments")) {
    std::fprintf(stderr, "recordd: create route failed path=%s error=%s\n",
                 route_path_.c_str(), std::strerror(errno));
    return false;
  }
  segment_index_ = 0;
  total_video_frames_ = 0;
  event_records_ = 0;
  event_file_ = open_buffered(route_path_ + "/events.bin");
  if (!event_file_) {
    std::fprintf(stderr, "recordd: open event log failed: %s\n", std::strerror(errno));
    return false;
  }
  K230EventFileHeader header;
  header.route_start_ns = now_ns;
  if (std::fwrite(&header, sizeof(header), 1, event_file_) != 1) {
    close_route(false);
    return false;
  }
  snapshot_params();
  write_manifest(false);
  next_storage_check_ns_ = now_ns + kStorageCheckIntervalNs;
  std::fprintf(stderr, "recordd: recording started route=%s\n", route_path_.c_str());
  return true;
}

void RecordingWriter::set_codec_config(const uint8_t *data, size_t size) {
  if (!data || size == 0) return;
  codec_config_.assign(data, data + size);
}

bool RecordingWriter::open_segment(const K230RoadAiFrame &frame) {
  std::ostringstream number;
  number << std::setw(3) << std::setfill('0') << segment_index_;
  const std::string directory = route_path_ + "/segments/" + number.str();
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
  if (!requested_enabled_ || !event_file_ || !data || size == 0) return;
  if (frame.timestamp_ns >= next_storage_check_ns_) {
    next_storage_check_ns_ = frame.timestamp_ns + kStorageCheckIntervalNs;
    if (!has_storage_reserve()) {
      blocked_for_space_ = true;
      requested_enabled_ = false;
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
  index.encode_index = total_video_frames_;
  index.file_offset = offset;
  index.packet_size = static_cast<uint32_t>(size);
  index.flags = keyframe ? 1U : 0U;
  if (std::fwrite(&index, sizeof(index), 1, index_file_) != 1) {
    std::fprintf(stderr, "recordd: frame index write failed: %s\n", std::strerror(errno));
    return;
  }
  video_offset_ += size;
  ++total_video_frames_;
}

bool RecordingWriter::write_event_header(K230RecordType type, uint64_t timestamp_ns,
                                          uint32_t payload_size) {
  if (!event_file_) return false;
  K230EventRecordHeader header;
  header.timestamp_ns = timestamp_ns;
  header.type = static_cast<uint16_t>(type);
  header.payload_size = payload_size;
  return std::fwrite(&header, sizeof(header), 1, event_file_) == 1;
}

void RecordingWriter::write_can(K230RecordType type, const K230CanBatch &batch) {
  if (!event_file_ || (type != K230RecordType::CanRx && type != K230RecordType::CanTx)) return;
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
  if (!event_file_ || !data || size == 0 || size > UINT32_MAX) return;
  if (!write_event_header(type, timestamp_ns, static_cast<uint32_t>(size))) return;
  if (std::fwrite(data, 1, size, event_file_) == size) ++event_records_;
}

void RecordingWriter::close_segment() {
  if (video_file_) std::fclose(video_file_);
  if (index_file_) std::fclose(index_file_);
  video_file_ = nullptr;
  index_file_ = nullptr;
  segment_start_ns_ = 0;
  video_offset_ = 0;
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
           << "  \"video_frames\": " << total_video_frames_ << ",\n"
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
  if (event_file_) std::fclose(event_file_);
  event_file_ = nullptr;
  write_manifest(complete);
  if (!route_path_.empty()) {
    std::fprintf(stderr,
                 "recordd: recording stopped route=%s frames=%llu events=%llu\n",
                 route_path_.c_str(),
                 static_cast<unsigned long long>(total_video_frames_),
                 static_cast<unsigned long long>(event_records_));
  }
  route_path_.clear();
  segment_index_ = 0;
}

void RecordingWriter::close() {
  requested_enabled_ = false;
  close_route(true);
}
