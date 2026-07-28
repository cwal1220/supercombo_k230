#!/usr/bin/env python3
import mmap
import os
import signal
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional


IPC_MAGIC = 0x4B323349
IPC_VERSION = 1
HEADER = struct.Struct("<IIIIQQII")
HEADER_SIZE = HEADER.size
PROCESS = struct.Struct("<16sIiiIQ")
MAX_PROCESSES = 6
MANAGER_STATE_SIZE = 8 + 4 + 4 + PROCESS.size * MAX_PROCESSES
DISPLAY_READY_FILE = "/tmp/k230_display_ready"
DISPLAY_READY_TIMEOUT_MS = 7000
START_ORDER = ("k230_overlay", "k230_camerad", "k230_modeld")
PROCESS_ORDER = ("k230_camerad", "k230_modeld", "k230_overlay")
DEFAULT_KMODEL_CANDIDATES = (
    "model/supercombo.kmodel",
    "models/supercombo.kmodel",
)
DEFAULT_DEBUG_MODE = "0"


def now_ns() -> int:
    if hasattr(time, "CLOCK_BOOTTIME"):
        return time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    return time.monotonic_ns()


def env_enabled(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value not in ("0", "false", "False", "FALSE")


def default_kmodel_path() -> str:
    override = os.environ.get("K230_KMODEL")
    if override:
        return override
    return next(
        (path for path in DEFAULT_KMODEL_CANDIDATES if os.path.exists(path)),
        DEFAULT_KMODEL_CANDIDATES[0],
    )


class LatestPublisher:
    def __init__(self, name: str, payload_size: int):
        path = "/dev/shm/" + name.lstrip("/")
        self.payload_size = payload_size
        self.fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o664)
        os.ftruncate(self.fd, HEADER_SIZE + payload_size)
        self.map = mmap.mmap(self.fd, HEADER_SIZE + payload_size)
        self._init_header_if_needed()

    def _read_header(self):
        self.map.seek(0)
        return HEADER.unpack(self.map.read(HEADER_SIZE))

    def _write_header(self, seq: int, timestamp_ns: int, payload_size: int):
        self.map.seek(0)
        self.map.write(HEADER.pack(
            IPC_MAGIC,
            IPC_VERSION,
            self.payload_size,
            0,
            seq,
            timestamp_ns,
            payload_size,
            0,
        ))

    def _init_header_if_needed(self):
        try:
            magic, version, capacity, _, seq, ts, size, _ = self._read_header()
        except struct.error:
            magic = version = capacity = seq = ts = size = 0
        if magic != IPC_MAGIC or version != IPC_VERSION or capacity != self.payload_size:
            self._write_header(0, 0, 0)

    def publish(self, payload: bytes):
        if len(payload) > self.payload_size:
            raise ValueError("payload too large")
        magic, version, capacity, reserved0, seq, ts, size, reserved1 = self._read_header()
        if seq & 1:
            seq += 1
        self._write_header(seq + 1, ts, size)
        self.map.seek(HEADER_SIZE)
        self.map.write(payload)
        if len(payload) < self.payload_size:
            self.map.write(b"\x00" * (self.payload_size - len(payload)))
        self._write_header(seq + 2, now_ns(), len(payload))

    def close(self):
        self.map.close()
        os.close(self.fd)


@dataclass
class ProcSpec:
    name: str
    cmd: List[str]
    nice: int = 0


@dataclass
class ProcState:
    spec: ProcSpec
    proc: Optional[subprocess.Popen] = None
    restart_count: int = 0
    last_start_ns: int = 0
    exit_code: int = 0
    next_restart_monotonic: float = 0.0

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None


def wait_for_file(path: str, timeout_ms: int) -> bool:
    deadline = time.monotonic() + max(0, timeout_ms) / 1000.0
    while time.monotonic() < deadline:
        if os.path.exists(path):
            return True
        time.sleep(0.02)
    return os.path.exists(path)


def child_setup(nice_adjust: int):
    os.setsid()
    if nice_adjust != 0:
        try:
            os.nice(nice_adjust)
        except OSError:
            pass


class Manager:
    def __init__(self, argv: List[str]):
        if len(argv) > 3:
            raise ValueError(
                f"Usage: {argv[0] if argv else 'k230_manager.py'} "
                "[supercombo.kmodel] [debug_mode]"
            )
        self.kmodel = argv[1] if len(argv) >= 2 else default_kmodel_path()
        self.debug = argv[2] if len(argv) >= 3 else DEFAULT_DEBUG_MODE
        os.environ.setdefault("K230_ENABLE_CONTROL", "1")
        os.environ.setdefault("K230_PANDA_TX", "1")
        os.environ.setdefault("K230_PANDA_ENGAGED", "1")
        os.environ.setdefault("K230_PANDA_SAFETY", "hyundaiCommunity")
        os.environ.setdefault("K230_PANDA_SAFETY_PARAM", "0")
        app_lib = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib")
        os.environ["LD_LIBRARY_PATH"] = app_lib + (
            ":" + os.environ["LD_LIBRARY_PATH"] if os.environ.get("LD_LIBRARY_PATH") else ""
        )
        self.shutdown = False
        self.manager_state = LatestPublisher("/k230_manager_state", MANAGER_STATE_SIZE)
        self.procs: Dict[str, ProcState] = {}

        self.start_order = list(START_ORDER)
        self.process_order = list(PROCESS_ORDER)
        enable_control = env_enabled("K230_ENABLE_CONTROL")
        specs = [
            ProcSpec("k230_camerad", ["./k230_camerad"], 0),
            ProcSpec("k230_modeld", ["./k230_modeld", self.kmodel, self.debug], -5),
            ProcSpec("k230_overlay", ["./k230_overlay"], 10),
        ]
        if env_enabled("K230_ENABLE_PANDA") or enable_control:
            specs.append(ProcSpec("k230_pandad", ["./k230_pandad"], -10))
            self.start_order.append("k230_pandad")
            self.process_order.append("k230_pandad")
        if enable_control:
            specs.append(ProcSpec("k230_controlsd", ["./k230_k7_controlsd"], -8))
            self.start_order.append("k230_controlsd")
            self.process_order.append("k230_controlsd")
        if env_enabled("K230_ENABLE_PARAM_SERVER", enable_control):
            server_script = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "k7_param_server.py"
            )
            specs.append(
                ProcSpec("k7_param_server", [sys.executable, server_script], 10)
            )
            self.start_order.append("k7_param_server")
            self.process_order.append("k7_param_server")
        for spec in specs:
            self.procs[spec.name] = ProcState(spec=spec)
        self.display_ready_file = DISPLAY_READY_FILE

    def start_proc(self, state: ProcState):
        if not os.path.exists(state.spec.cmd[0]):
            raise FileNotFoundError(f"{state.spec.cmd[0]} not found")
        state.proc = subprocess.Popen(
            state.spec.cmd,
            preexec_fn=lambda nice=state.spec.nice: child_setup(nice),
        )
        state.last_start_ns = now_ns()
        state.exit_code = 0
        print(f"manager: started {state.spec.name} pid={state.proc.pid} nice={state.spec.nice} "
              f"cmd={' '.join(state.spec.cmd)}",
              flush=True)

    def terminate_proc(self, state: ProcState, sig=signal.SIGTERM):
        if not state.running():
            return
        try:
            os.killpg(os.getpgid(state.proc.pid), sig)
        except ProcessLookupError:
            pass

    def publish_state(self):
        timestamp = now_ns()
        payload = struct.pack("<QII", timestamp, min(len(self.procs), MAX_PROCESSES), 0)
        for name in self.process_order[:MAX_PROCESSES]:
            state = self.procs[name]
            proc_name = name.encode("ascii")[:15].ljust(16, b"\x00")
            pid = state.proc.pid if state.proc is not None else 0
            running = 1 if state.running() else 0
            exit_code = state.exit_code
            payload += PROCESS.pack(proc_name, running, pid, exit_code,
                                    state.restart_count, state.last_start_ns)
        payload += b"\x00" * (MANAGER_STATE_SIZE - len(payload))
        self.manager_state.publish(payload)

    def handle_signal(self, signum, _frame):
        print(f"\nmanager: signal {signum}, stopping children", flush=True)
        self.shutdown = True
        for state in self.procs.values():
            self.terminate_proc(state, signal.SIGTERM)

    def run(self) -> int:
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, self.handle_signal)

        try:
            os.unlink(self.display_ready_file)
        except FileNotFoundError:
            pass
        except OSError as exc:
            print(f"manager: failed to remove display ready file {self.display_ready_file}: {exc}",
                  flush=True)

        for name in self.start_order:
            if name == "k230_camerad":
                self.wait_for_display_ready()
            self.start_proc(self.procs[name])
            time.sleep(0.3)

        last_publish = 0.0
        exit_code = 0
        while not self.shutdown:
            now = time.monotonic()
            if now - last_publish >= 1.0:
                self.publish_state()
                last_publish = now

            for state in self.procs.values():
                if state.proc is None:
                    continue
                ret = state.proc.poll()
                if ret is None:
                    continue
                state.exit_code = ret
                print(f"\nmanager: {state.spec.name} exited code={ret}", flush=True)
                state.proc = None
                exit_code = ret if ret != 0 else exit_code
                state.restart_count += 1
                state.next_restart_monotonic = now + 1.0

            for state in self.procs.values():
                if state.proc is not None or self.shutdown:
                    continue
                if now >= state.next_restart_monotonic:
                    self.start_proc(state)

            time.sleep(0.1)

        for state in self.procs.values():
            self.terminate_proc(state, signal.SIGTERM)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if all(not state.running() for state in self.procs.values()):
                break
            time.sleep(0.1)
        for state in self.procs.values():
            self.terminate_proc(state, signal.SIGKILL)
        self.publish_state()
        self.manager_state.close()
        return exit_code

    def wait_for_display_ready(self):
        print(f"manager: waiting for display ready {self.display_ready_file}",
              flush=True)
        if wait_for_file(self.display_ready_file, DISPLAY_READY_TIMEOUT_MS):
            print("manager: display ready, starting camera/model pipeline",
                  flush=True)
        else:
            print(f"manager: display ready timeout after {DISPLAY_READY_TIMEOUT_MS}ms, "
                  "starting camera/model pipeline anyway",
                  flush=True)


def main(argv: List[str]) -> int:
    try:
        return Manager(argv).run()
    except Exception as exc:
        print(f"manager error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
