#!/usr/bin/env python3
import mmap
import os
import signal
import struct
import sys
import time
import traceback
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple
from types import ModuleType, SimpleNamespace


IPC_MAGIC = 0x4B323349
IPC_VERSION = 1
HEADER = struct.Struct("<IIIIQQII")
HEADER_SIZE = HEADER.size

CAN_FRAME = struct.Struct("<IIIII64s")
CAN_BATCH_HEADER = struct.Struct("<QIIII")
CAN_BATCH_MAX_FRAMES = 256
CAN_BATCH_SIZE = CAN_BATCH_HEADER.size + CAN_FRAME.size * CAN_BATCH_MAX_FRAMES
PANDA_STATE = struct.Struct("<Q" + "I" * 16)
LATERAL_TARGET = struct.Struct("<Iffff")

CAN_TOPIC = "/k230_can"
SENDCAN_TOPIC = "/k230_sendcan"
MODEL_STATE_TOPIC = "/k230_model_state"
PANDA_STATE_TOPIC = "/k230_panda_state"


PARAM_DEFAULTS = {
    "AutoRESDelay": "0",
    "AutoResCondition": "0",
    "AutoResLimitTime": "0",
    "AutoResOption": "0",
    "AvoidLKASFaultBeyond": "0",
    "AvoidLKASFaultEnabled": "0",
    "AvoidLKASFaultMaxAngle": "90",
    "AvoidLKASFaultMaxFrame": "89",
    "CruiseAutoRes": "0",
    "CruiseGapAdjust": "0",
    "CruiseGapBySpdGap": "4,3,2",
    "CruiseGapBySpdOn": "0",
    "CruiseGapBySpdSpd": "30,60,90",
    "CruiseStatemodeSelInit": "0",
    "CurvDecelOption": "0",
    "DepartChimeAtResume": "0",
    "E2ELong": "0",
    "FCA11Message": "0",
    "FingerprintTwoSet": "0",
    "JoystickDebugMode": "0",
    "JustDoGearD": "0",
    "LdwsCarFix": "0",
    "NoSmartMDPS": "0",
    "OCurvSpeedC": "30,60,90",
    "OCurvSpeedT": "30,60,90",
    "OPKRSpeedBump": "0",
    "OPKRNaviSelect": "0",
    "OSMCustomSpeedLimitC": "30,60,90",
    "OSMCustomSpeedLimitT": "30,60,90",
    "OSMSpeedLimitEnable": "0",
    "OpkrAutoResume": "0",
    "OpkrDriverAngleWait": "0.001",
    "OpkrLiveTunePanelEnable": "0",
    "OpkrMapEnable": "0",
    "OpkrMaxAngleLimit": "90",
    "OpkrMaxDriverAngleWait": "0.002",
    "OpkrMaxSteerAngleWait": "0.001",
    "OpkrMaxSteeringAngle": "90",
    "OpkrSpeedLimitOffset": "0",
    "OpkrSpeedLimitOffsetOption": "0",
    "OpkrSteerAngleCorrection": "0",
    "OpkrSteerMethod": "1",
    "OpkrTurnSteeringDisable": "0",
    "OpkrVariableCruise": "0",
    "OpkrVariableSteerDelta": "0",
    "OpkrVariableSteerMax": "0",
    "RadarDisable": "0",
    "RadarLongHelper": "0",
    "RESCountatStandstill": "25",
    "RoadList": "\n",
    "RoutineDriveOn": "0",
    "RoutineDriveOption": "000",
    "SafetyCamDecelDistGain": "0",
    "SetSpeedFive": "0",
    "SpeedLimitDecelOff": "1",
    "StandstillResumeAlt": "0",
    "SteerDeltaDownAdj": "7",
    "SteerDeltaDownBaseAdj": "7",
    "SteerDeltaUpAdj": "3",
    "SteerDeltaUpBaseAdj": "3",
    "SteerMaxAdj": "384",
    "SteerMaxBaseAdj": "384",
    "SteerThreshold": "150",
    "SteerWarningFix": "0",
    "StockNaviSpeedEnabled": "0",
    "StopAtStopSign": "0",
    "StoppingDist": "0",
    "StoppingDistAdj": "0",
    "UseStockDecelOnSS": "0",
    "UFCModeEnabled": "0",
    "UseRadarTrack": "0",
    "UserSpecificFeature": "0",
    "VCurvSpeedC": "30,60,90",
    "VCurvSpeedCMPH": "20,40,60",
    "VCurvSpeedT": "30,60,90",
    "VCurvSpeedTMPH": "20,40,60",
}


def ipc_path(name: str) -> str:
    base = os.environ.get("K230_IPC_DIR", "/dev/shm")
    os.makedirs(base, exist_ok=True)
    return os.path.join(base, name.lstrip("/"))


def now_ns() -> int:
    if hasattr(time, "CLOCK_BOOTTIME"):
        return time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    return time.monotonic_ns()


def env_enabled(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value not in ("0", "false", "False", "FALSE")


def env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, ""))
    except ValueError:
        return default


class DummyMessage(SimpleNamespace):
    def __getattr__(self, _name: str):
        return 0


class DummySubMaster:
    def __init__(self, services):
        self.data = {service: self._make_service(service) for service in services}

    @staticmethod
    def _make_service(service: str):
        if service == "controlsState":
            return DummyMessage(vCruise=0.0, longControlState=0, curvature=0.0, pauseSpdLimit=False)
        if service == "radarState":
            lead = DummyMessage(dRel=0.0, vRel=0.0, yRel=0.0, status=False)
            return DummyMessage(leadOne=lead, leadTwo=lead)
        if service == "longitudinalPlan":
            return DummyMessage(e2eX=[0.0] * 33, stopLine=[200.0] * 33, longitudinalPlanSource=0)
        if service == "liveMapData":
            return DummyMessage(speedLimit=0.0, speedLimitAhead=0.0, speedLimitAheadDistance=0.0,
                                currentRoadName="", turnSpeedLimit=0.0, turnSpeedLimitEndDistance=0.0)
        if service in ("liveNaviData", "liveENaviData"):
            return DummyMessage(wazeRoadSpeedLimit=0.0, wazeAlertDistance=0.0)
        return DummyMessage()

    def update(self, _timeout: int = 0):
        return None

    def __getitem__(self, service: str):
        return self.data.setdefault(service, self._make_service(service))


class K230Params:
    def get(self, key: str, encoding: Optional[str] = None):
        value = os.environ.get(f"K230_PARAM_{key}", PARAM_DEFAULTS.get(key, "0"))
        if encoding is None:
            return value.encode("utf8")
        return value

    def get_bool(self, key: str) -> bool:
        value = os.environ.get(f"K230_PARAM_{key}", PARAM_DEFAULTS.get(key, "0"))
        return value not in ("", "0", "false", "False", "FALSE")

    def put(self, key: str, value):
        os.environ[f"K230_PARAM_{key}"] = value.decode("utf8") if isinstance(value, bytes) else str(value)

    def put_bool(self, key: str, value: bool):
        self.put(key, "1" if value else "0")

    def check_key(self, _key: str) -> bool:
        return True


class K230KF1D:
    def __init__(self, x0, A, C, K):
        self.x = [[float(x0[0][0])], [float(x0[1][0])]]
        self.A = A
        self.C = C
        self.K = K

    def update(self, meas):
        pred0 = self.A[0][0] * self.x[0][0] + self.A[0][1] * self.x[1][0]
        pred1 = self.A[1][0] * self.x[0][0] + self.A[1][1] * self.x[1][0]
        err = float(meas) - (self.C[0] * pred0 + self.C[1] * pred1)
        self.x = [[pred0 + self.K[0][0] * err], [pred1 + self.K[1][0] * err]]
        return [self.x[0][0], self.x[1][0]]


def install_openpilot_shims():
    clock_mod = ModuleType("common.clock")
    clock_mod.sec_since_boot = lambda: time.monotonic()
    sys.modules["common.clock"] = clock_mod

    kalman_impl_mod = ModuleType("common.kalman.simple_kalman_impl")
    kalman_impl_mod.KF1D = K230KF1D
    sys.modules["common.kalman.simple_kalman_impl"] = kalman_impl_mod

    hardware_mod = ModuleType("selfdrive.hardware")
    hardware_mod.PC = True
    hardware_mod.TICI = False
    hardware_mod.EON = False
    hardware_mod.HARDWARE = DummyMessage()
    sys.modules["selfdrive.hardware"] = hardware_mod

    longcontrol_mod = ModuleType("selfdrive.controls.lib.longcontrol")
    longcontrol_mod.LongCtrlState = SimpleNamespace(off=0, pid=1, stopping=2, starting=3)
    sys.modules["selfdrive.controls.lib.longcontrol"] = longcontrol_mod

    desire_helper_mod = ModuleType("selfdrive.controls.lib.desire_helper")
    desire_helper_mod.LANE_CHANGE_SPEED_MIN = -1.0
    sys.modules["selfdrive.controls.lib.desire_helper"] = desire_helper_mod

    class K230Events:
        def __init__(self):
            self.events = []

        def add(self, event):
            self.events.append(event)

        def to_msg(self):
            return []

    events_mod = ModuleType("selfdrive.controls.lib.events")
    events_mod.Events = K230Events
    sys.modules["selfdrive.controls.lib.events"] = events_mod

    disable_ecu_mod = ModuleType("selfdrive.car.disable_ecu")
    disable_ecu_mod.disable_ecu = lambda *args, **kwargs: None
    sys.modules["selfdrive.car.disable_ecu"] = disable_ecu_mod

    swaglog_mod = ModuleType("selfdrive.swaglog")
    swaglog_mod.cloudlog = DummyMessage(debug=lambda *a, **k: None,
                                        info=lambda *a, **k: None,
                                        warning=lambda *a, **k: None,
                                        error=lambda *a, **k: None,
                                        exception=lambda *a, **k: None,
                                        event=lambda *a, **k: None)
    sys.modules["selfdrive.swaglog"] = swaglog_mod

    params_mod = ModuleType("common.params")
    params_mod.Params = K230Params
    params_mod.ParamKeyType = SimpleNamespace()
    params_mod.UnknownKeyName = KeyError
    params_mod.put_nonblocking = lambda key, value: K230Params().put(key, value)
    sys.modules["common.params"] = params_mod

    params_pyx_mod = ModuleType("common.params_pyx")
    params_pyx_mod.Params = K230Params
    params_pyx_mod.ParamKeyType = SimpleNamespace()
    params_pyx_mod.UnknownKeyName = KeyError
    params_pyx_mod.put_nonblocking = params_mod.put_nonblocking
    sys.modules["common.params_pyx"] = params_pyx_mod

    messaging_mod = ModuleType("cereal.messaging")
    messaging_mod.SubMaster = DummySubMaster
    sys.modules["cereal.messaging"] = messaging_mod

    panda_mod = ModuleType("panda")
    panda_mod.Panda = type("Panda", (), {})
    sys.modules.setdefault("panda", panda_mod)

    cereal_pkg = sys.modules.get("cereal")
    if cereal_pkg is not None:
        setattr(cereal_pkg, "messaging", messaging_mod)


class LatestChannel:
    def __init__(self, name: str, payload_size: int, create: bool):
        self.name = name
        self.payload_size = payload_size
        path = ipc_path(name)
        flags = os.O_RDWR | (os.O_CREAT if create else 0)
        self.fd = os.open(path, flags, 0o664)
        if create:
            os.ftruncate(self.fd, HEADER_SIZE + payload_size)
            self.map_size = HEADER_SIZE + payload_size
        else:
            self.map_size = os.fstat(self.fd).st_size
            if self.map_size < HEADER_SIZE:
                raise RuntimeError(f"{name} is too small")
        self.map = mmap.mmap(self.fd, self.map_size)
        if create:
            self._init_header()
        else:
            magic, version, capacity, *_ = self._read_header()
            if magic != IPC_MAGIC or version != IPC_VERSION or capacity < payload_size:
                raise RuntimeError(f"{name} has incompatible IPC header")

    def _read_header(self):
        self.map.seek(0)
        return HEADER.unpack(self.map.read(HEADER_SIZE))

    def _write_header(self, seq: int, timestamp_ns: int, payload_size: int):
        self.map.seek(0)
        self.map.write(HEADER.pack(IPC_MAGIC, IPC_VERSION, self.payload_size, 0,
                                   seq, timestamp_ns, payload_size, 0))

    def _init_header(self):
        try:
            magic, version, capacity, _, seq, ts, size, _ = self._read_header()
        except struct.error:
            magic = version = capacity = seq = ts = size = 0
        if magic != IPC_MAGIC or version != IPC_VERSION or capacity != self.payload_size:
            self._write_header(0, 0, 0)
            self.map.seek(HEADER_SIZE)
            self.map.write(b"\x00" * self.payload_size)

    def publish(self, payload: bytes):
        if len(payload) > self.payload_size:
            raise ValueError(f"{self.name} payload too large")
        magic, version, capacity, _, seq, ts, size, _ = self._read_header()
        if magic != IPC_MAGIC or version != IPC_VERSION or capacity < len(payload):
            raise RuntimeError(f"{self.name} has incompatible IPC header")
        if seq & 1:
            seq += 1
        self._write_header(seq + 1, ts, size)
        self.map.seek(HEADER_SIZE)
        self.map.write(payload)
        if len(payload) < self.payload_size:
            self.map.write(b"\x00" * (self.payload_size - len(payload)))
        self._write_header(seq + 2, now_ns(), len(payload))

    def read(self) -> Tuple[int, bytes]:
        for _ in range(4):
            magic, version, capacity, _, before, _, size, _ = self._read_header()
            if magic != IPC_MAGIC or version != IPC_VERSION or before == 0 or before & 1:
                return 0, b""
            if size == 0 or size > capacity or HEADER_SIZE + size > self.map_size:
                return 0, b""
            self.map.seek(HEADER_SIZE)
            payload = self.map.read(size)
            after = self._read_header()[4]
            if before == after and not (after & 1):
                return after, payload
        return 0, b""

    def read_new(self, last_seq: int, timeout_ms: int) -> Tuple[int, bytes]:
        deadline = time.monotonic() + max(0, timeout_ms) / 1000.0
        while True:
            seq, payload = self.read()
            if seq and seq != last_seq:
                return seq, payload
            if timeout_ms == 0 or time.monotonic() >= deadline:
                return last_seq, b""
            time.sleep(0.001)

    def close(self):
        self.map.close()
        os.close(self.fd)


@dataclass
class CanFrame:
    address: int
    src: int
    dat: bytes


@dataclass
class LateralTarget:
    valid: bool = False
    lookahead_x: float = 0.0
    target_y: float = 0.0
    heading: float = 0.0
    curvature: float = 0.0


def decode_can_batch(payload: bytes) -> List[CanFrame]:
    if len(payload) < CAN_BATCH_HEADER.size:
        return []
    _timestamp_ns, valid, count, _dropped, _reserved = CAN_BATCH_HEADER.unpack_from(payload, 0)
    if not valid:
        return []
    frames: List[CanFrame] = []
    offset = CAN_BATCH_HEADER.size
    for _ in range(min(count, CAN_BATCH_MAX_FRAMES)):
        address, src, _bus_time, data_len, _flags, raw = CAN_FRAME.unpack_from(payload, offset)
        offset += CAN_FRAME.size
        if data_len <= len(raw):
            frames.append(CanFrame(address=address, src=src, dat=raw[:data_len]))
    return frames


def encode_can_batch(frames: Iterable[CanFrame]) -> bytes:
    frame_list = list(frames)[:CAN_BATCH_MAX_FRAMES]
    payload = bytearray(CAN_BATCH_SIZE)
    CAN_BATCH_HEADER.pack_into(payload, 0, now_ns(), 1, len(frame_list), 0, 0)
    offset = CAN_BATCH_HEADER.size
    for frame in frame_list:
        dat = bytes(frame.dat[:64])
        CAN_FRAME.pack_into(payload, offset, frame.address, frame.src, 0, len(dat), 0,
                            dat.ljust(64, b"\x00"))
        offset += CAN_FRAME.size
    return bytes(payload)


def decode_lateral_target(payload: bytes) -> LateralTarget:
    if len(payload) < LATERAL_TARGET.size:
        return LateralTarget()
    valid, lookahead_x, target_y, heading, curvature = LATERAL_TARGET.unpack_from(
        payload, len(payload) - LATERAL_TARGET.size)
    return LateralTarget(valid=bool(valid), lookahead_x=lookahead_x,
                         target_y=target_y, heading=heading, curvature=curvature)


def add_openpilot_to_path():
    candidates = [
        os.environ.get("K230_OPENPILOT_PATH", ""),
        "/root/openpilot_c2",
        "/data/openpilot",
        "/data/openpilot_c2",
        os.path.expanduser("~/openpilot_c2"),
    ]
    for path in candidates:
        if path and os.path.isdir(path):
            sys.path.insert(0, path)
            os.chdir(path)
            os.environ["PWD"] = path
            return path
    raise RuntimeError("openpilot path not found; set K230_OPENPILOT_PATH")


class OpenpilotHyundaiController:
    def __init__(self, fingerprint=None):
        self.openpilot_path = add_openpilot_to_path()
        install_openpilot_shims()
        from cereal import car, log
        from selfdrive.car import gen_empty_fingerprint
        from selfdrive.car.hyundai.carcontroller import CarController
        from selfdrive.car.hyundai.carstate import CarState
        from selfdrive.car.hyundai.interface import CarInterface
        from selfdrive.car.hyundai.values import CAR

        self.car = car
        self.log = log
        self.CI_cls = CarInterface
        self.candidate = CAR.K7_HEV_YG
        self.fingerprint = fingerprint if fingerprint is not None else gen_empty_fingerprint()
        self.CP = CarInterface.get_params(self.candidate, self.fingerprint)
        self.CI = CarInterface(self.CP, CarController, CarState)
        self.enabled = env_enabled("K230_CONTROLD_ENABLED", True)
        self.steer_gain = env_float("K230_CONTROLD_CURVATURE_GAIN", 4.0)
        self.last_actuators = None

    def can_strings(self, frames: List[CanFrame]) -> List[bytes]:
        if not frames:
            return []
        msg = self.log.Event.new_message()
        msg.logMonoTime = now_ns()
        can = msg.init("can", len(frames))
        for i, frame in enumerate(frames):
            can[i].address = frame.address
            can[i].src = frame.src
            can[i].busTime = 0
            can[i].dat = frame.dat
        return [msg.to_bytes()]

    def steer_from_target(self, target: LateralTarget) -> float:
        if not target.valid:
            return 0.0
        steer = -target.curvature * self.steer_gain
        return max(-1.0, min(1.0, steer))

    def update(self, frames: List[CanFrame], target: LateralTarget) -> List[CanFrame]:
        can_strings = self.can_strings(frames)
        cc = self.car.CarControl.new_message()
        car_state = self.CI.update(cc, can_strings)

        cc.enabled = bool(self.enabled)
        cc.active = bool(self.enabled and target.valid and car_state.cruiseState.enabled)
        cc.actuators.steer = self.steer_from_target(target)
        cc.actuators.accel = 0.0
        cc.hudControl.leftLaneVisible = True
        cc.hudControl.rightLaneVisible = True
        cc.hudControl.leadVisible = False
        cc.hudControl.setSpeed = 0.0
        cc.hudControl.vFuture = 0.0
        cc.hudControl.vFutureA = 0.0

        self.last_actuators, can_sends, *_ = self.CI.apply(cc)
        out: List[CanFrame] = []
        for msg in can_sends:
            if len(msg) < 4:
                continue
            address = int(msg[0])
            dat = bytes(msg[2])
            bus = int(msg[3])
            if not (0 <= bus < 4 and 0 <= address <= 0x1FFFFFFF and len(dat) <= 64):
                continue
            out.append(CanFrame(address=address, src=bus, dat=dat))
        return out


def wait_for_channel(name: str, payload_size: int) -> LatestChannel:
    last_log = 0.0
    while True:
        try:
            return LatestChannel(name, payload_size, create=False)
        except FileNotFoundError:
            now = time.monotonic()
            if now - last_log >= 1.0:
                print(f"k230_controlsd: waiting for {name}", flush=True)
                last_log = now
            time.sleep(0.1)


def empty_fingerprint():
    return {i: {} for i in range(4)}


def add_frames_to_fingerprint(fingerprint, frames: List[CanFrame]):
    for frame in frames:
        if 0 <= frame.src < 4 and frame.address < 0x800:
            fingerprint[frame.src][frame.address] = len(frame.dat)


def fingerprint_addr_count(fingerprint) -> int:
    return sum(len(bus) for bus in fingerprint.values())


def main() -> int:
    stop = False

    def handle_signal(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        can_sub = wait_for_channel(CAN_TOPIC, CAN_BATCH_SIZE)
        model_sub = wait_for_channel(MODEL_STATE_TOPIC, LATERAL_TARGET.size)
        sendcan_pub = LatestChannel(SENDCAN_TOPIC, CAN_BATCH_SIZE, create=True)
        panda_state_sub: Optional[LatestChannel] = None
        try:
            panda_state_sub = LatestChannel(PANDA_STATE_TOPIC, PANDA_STATE.size, create=False)
        except FileNotFoundError:
            panda_state_sub = None

        controller: Optional[OpenpilotHyundaiController] = None
        fingerprint = empty_fingerprint()
        fp_start = 0.0
        fp_seconds = env_float("K230_CONTROLD_FINGERPRINT_SEC", 2.0)
        fp_min_addrs = int(env_float("K230_CONTROLD_FINGERPRINT_MIN_ADDRS", 20.0))
        last_can_seq = 0
        last_model_seq = 0
        target = LateralTarget()
        frames_in = 0
        frames_out = 0
        errors = 0
        last_log = time.monotonic()

        while not stop:
            last_model_seq, model_payload = model_sub.read_new(last_model_seq, 0)
            if model_payload:
                target = decode_lateral_target(model_payload)

            last_can_seq, can_payload = can_sub.read_new(last_can_seq, 1000)
            if not can_payload:
                continue

            frames = decode_can_batch(can_payload)
            frames_in += len(frames)
            if controller is None:
                if frames and fp_start == 0.0:
                    fp_start = time.monotonic()
                add_frames_to_fingerprint(fingerprint, frames)
                fp_age = time.monotonic() - fp_start if fp_start else 0.0
                if fp_start and (fp_age >= fp_seconds or fingerprint_addr_count(fingerprint) >= fp_min_addrs):
                    controller = OpenpilotHyundaiController(fingerprint)
                    safety = controller.CP.safetyConfigs[0]
                    print(f"k230_controlsd: openpilot={controller.openpilot_path} "
                          f"candidate={controller.candidate} enabled={int(controller.enabled)} "
                          f"gain={controller.steer_gain} sccBus={controller.CP.sccBus} "
                          f"mdpsBus={controller.CP.mdpsBus} sasBus={controller.CP.sasBus} "
                          f"safety={safety.safetyModel}:{safety.safetyParam} "
                          f"fingerprint_addrs={fingerprint_addr_count(fingerprint)}", flush=True)
                continue

            try:
                out_frames = controller.update(frames, target)
                if out_frames:
                    sendcan_pub.publish(encode_can_batch(out_frames))
                    frames_out += len(out_frames)
            except Exception as exc:  # keep the shadow process alive while CAN state warms up
                errors += 1
                if errors <= 5:
                    print(f"k230_controlsd: controller update failed: {exc}", flush=True)
                    if env_enabled("K230_CONTROLD_TRACEBACK"):
                        traceback.print_exc()

            now = time.monotonic()
            if now - last_log >= 1.0:
                controls_allowed = 0
                if panda_state_sub is not None:
                    _seq, panda_payload = panda_state_sub.read()
                    if len(panda_payload) >= PANDA_STATE.size:
                        unpacked = PANDA_STATE.unpack_from(panda_payload, 0)
                        controls_allowed = unpacked[4]
                print(f"k230_controlsd: can_in={frames_in} sendcan={frames_out} "
                      f"errors={errors} lateral={int(target.valid)} "
                      f"controls_allowed={controls_allowed} "
                      f"fingerprint_addrs={fingerprint_addr_count(fingerprint)}", flush=True)
                frames_in = frames_out = errors = 0
                last_log = now

        print("\nk230_controlsd: stopping", flush=True)
        return 0
    except Exception as exc:
        print(f"k230_controlsd error: {exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    sys.exit(main())
