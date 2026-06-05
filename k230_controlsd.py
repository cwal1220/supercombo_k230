#!/usr/bin/env python3
import math
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
CAN_DLC_LENGTHS = frozenset((0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64))
CAN_MAX_ADDRESS = 0x1FFFFFFF
CAN_MAX_TX_BUS = 3
PANDA_STATE = struct.Struct("<Q" + "I" * 16)
TRAJECTORY_SIZE = 33
CONTROL_N = 17
DESIRE_LEN = 8
MODEL_HEADER = struct.Struct("<QQQfIIif")
POINT_FLOATS = 3
LATERAL_TARGET = struct.Struct("<Iffff")
LATERAL_PLAN = struct.Struct("<II" + "f" * (CONTROL_N * 4 + 1) + "I")

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
    "CameraOffsetAdj": "0",
    "CloseToRoadEdge": "0",
    "CruiseAutoRes": "0",
    "CruiseGapAdjust": "0",
    "CruiseGapBySpdGap": "4,3,2",
    "CruiseGapBySpdOn": "0",
    "CruiseGapBySpdSpd": "30,60,90",
    "CruiseStatemodeSelInit": "0",
    "CurvDecelOption": "0",
    "DepartChimeAtResume": "0",
    "DesiredCurvatureLimit": "10",
    "E2ELong": "0",
    "FCA11Message": "0",
    "FingerprintTwoSet": "0",
    "IsMetric": "1",
    "JoystickDebugMode": "0",
    "JustDoGearD": "0",
    "LaneWidth": "37",
    "LanelessMode": "0",
    "LdwsCarFix": "0",
    "LateralControlMethod": "3",
    "LCTimingFactor30": "100",
    "LCTimingFactor60": "100",
    "LCTimingFactor80": "100",
    "LCTimingFactor110": "100",
    "LCTimingFactorEnable": "0",
    "LeftCurvOffsetAdj": "0",
    "LeftEdgeOffset": "0",
    "LqrKi": "16",
    "EndToEndToggle": "0",
    "MultipleLateralUse": "0",
    "MultipleLateralOpS": "0,1,3",
    "MultipleLateralSpd": "30,60,90",
    "MultipleLateralOpA": "0,1,3",
    "MultipleLateralAng": "10,30,60",
    "PathOffsetAdj": "0",
    "PidKd": "150",
    "PidKf": "7",
    "PidKi": "40",
    "PidKp": "25",
    "NoSmartMDPS": "0",
    "OCurvSpeedC": "30,60,90",
    "OCurvSpeedT": "30,60,90",
    "OPKRSpeedBump": "0",
    "OPKRNaviSelect": "0",
    "OSMCustomSpeedLimitC": "30,60,90",
    "OSMCustomSpeedLimitT": "30,60,90",
    "OSMSpeedLimitEnable": "0",
    "OpkrAutoResume": "0",
    "OpkrAutoLaneChangeDelay": "0",
    "OpkrDriverAngleWait": "0.001",
    "OpkrLaneChangeSpeed": "0",
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
    "RightCurvOffsetAdj": "0",
    "RightEdgeOffset": "0",
    "Scale": "1500",
    "SafetyCamDecelDistGain": "0",
    "SetSpeedFive": "0",
    "SpeedLimitDecelOff": "1",
    "SpeedCameraOffset": "0",
    "SpdLaneWidthSet": "2.8,3.5",
    "SpdLaneWidthSpd": "0,31",
    "StandstillResumeAlt": "0",
    "SteerDeltaDownAdj": "7",
    "SteerDeltaDownBaseAdj": "7",
    "SteerDeltaUpAdj": "3",
    "SteerDeltaUpBaseAdj": "3",
    "SteerActuatorDelayAdj": "36",
    "SteerLimitTimerAdj": "100",
    "SteerMaxAdj": "384",
    "SteerMaxBaseAdj": "384",
    "SteerRatioAdj": "1550",
    "SteerRatioMaxAdj": "1750",
    "SteerThreshold": "150",
    "SteerWarningFix": "0",
    "StockNaviSpeedEnabled": "0",
    "StopAtStopSign": "0",
    "StoppingDist": "0",
    "StoppingDistAdj": "0",
    "UseStockDecelOnSS": "0",
    "TimeConstant": "14",
    "TireStiffnessFactorAdj": "85",
    "TorqueAngDeadZone": "10",
    "TorqueFriction": "65",
    "TorqueKf": "10",
    "TorqueKi": "1",
    "TorqueKp": "10",
    "TorqueMaxLatAccel": "27",
    "TorqueUseAngle": "1",
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


@dataclass
class LateralPlan:
    valid: bool = False
    mpc_solution_valid: bool = False
    psis: List[float] = None
    curvatures: List[float] = None
    curvature_rates: List[float] = None
    d_path_points: List[float] = None
    output_scale: float = 0.0

    def __post_init__(self):
        if self.psis is None:
            self.psis = [0.0] * CONTROL_N
        if self.curvatures is None:
            self.curvatures = [0.0] * CONTROL_N
        if self.curvature_rates is None:
            self.curvature_rates = [0.0] * CONTROL_N
        if self.d_path_points is None:
            self.d_path_points = [0.0] * CONTROL_N


@dataclass
class ModelPoint:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class K230ModelSnapshot:
    frame_id: int = 0
    capture_timestamp_ns: int = 0
    model_timestamp_ns: int = 0
    valid: bool = False
    model_t: List[float] = None
    lane_t: List[float] = None
    plan: List[ModelPoint] = None
    plan_stds: List[ModelPoint] = None
    plan_orientations: List[ModelPoint] = None
    lanes: List[List[ModelPoint]] = None
    lane_probabilities: List[float] = None
    lane_stds: List[float] = None
    road_edges: List[List[ModelPoint]] = None
    road_edge_stds: List[float] = None
    desire_state: List[float] = None

    def __post_init__(self):
        if self.model_t is None:
            self.model_t = [0.0] * TRAJECTORY_SIZE
        if self.lane_t is None:
            self.lane_t = [0.0] * TRAJECTORY_SIZE
        if self.plan is None:
            self.plan = [ModelPoint() for _ in range(TRAJECTORY_SIZE)]
        if self.plan_stds is None:
            self.plan_stds = [ModelPoint() for _ in range(TRAJECTORY_SIZE)]
        if self.plan_orientations is None:
            self.plan_orientations = [ModelPoint() for _ in range(TRAJECTORY_SIZE)]
        if self.lanes is None:
            self.lanes = [[ModelPoint() for _ in range(TRAJECTORY_SIZE)] for _ in range(4)]
        if self.lane_probabilities is None:
            self.lane_probabilities = [0.0] * 4
        if self.lane_stds is None:
            self.lane_stds = [0.0] * 4
        if self.road_edges is None:
            self.road_edges = [[ModelPoint() for _ in range(TRAJECTORY_SIZE)] for _ in range(2)]
        if self.road_edge_stds is None:
            self.road_edge_stds = [0.0] * 2
        if self.desire_state is None:
            self.desire_state = [0.0] * DESIRE_LEN


def unpack_float_list(payload: bytes, offset: int, count: int) -> Tuple[List[float], int]:
    values = list(struct.unpack_from("<" + "f" * count, payload, offset))
    return values, offset + count * 4


def unpack_points(payload: bytes, offset: int, count: int) -> Tuple[List[ModelPoint], int]:
    values, offset = unpack_float_list(payload, offset, count * POINT_FLOATS)
    points = [ModelPoint(values[i], values[i + 1], values[i + 2])
              for i in range(0, len(values), POINT_FLOATS)]
    return points, offset


def decode_k230_model_state(payload: bytes) -> Optional[K230ModelSnapshot]:
    min_size = MODEL_HEADER.size
    if len(payload) < min_size:
        return None

    frame_id, capture_ts, model_ts, _model_ms, valid, _projection_mode, _best_plan, _plan_prob = (
        MODEL_HEADER.unpack_from(payload, 0))
    offset = MODEL_HEADER.size

    try:
        model_t, offset = unpack_float_list(payload, offset, TRAJECTORY_SIZE)
        lane_t, offset = unpack_float_list(payload, offset, TRAJECTORY_SIZE)
        plan, offset = unpack_points(payload, offset, TRAJECTORY_SIZE)
        plan_stds, offset = unpack_points(payload, offset, TRAJECTORY_SIZE)
        plan_orientations, offset = unpack_points(payload, offset, TRAJECTORY_SIZE)
        lanes = []
        for _ in range(4):
            lane, offset = unpack_points(payload, offset, TRAJECTORY_SIZE)
            lanes.append(lane)
        lane_probabilities, offset = unpack_float_list(payload, offset, 4)
        lane_stds, offset = unpack_float_list(payload, offset, 4)
        road_edges = []
        for _ in range(2):
            edge, offset = unpack_points(payload, offset, TRAJECTORY_SIZE)
            road_edges.append(edge)
        road_edge_stds, offset = unpack_float_list(payload, offset, 2)
        desire_state, offset = unpack_float_list(payload, offset, DESIRE_LEN)
    except struct.error:
        return None

    return K230ModelSnapshot(frame_id=frame_id,
                             capture_timestamp_ns=capture_ts,
                             model_timestamp_ns=model_ts,
                             valid=bool(valid),
                             model_t=model_t,
                             lane_t=lane_t,
                             plan=plan,
                             plan_stds=plan_stds,
                             plan_orientations=plan_orientations,
                             lanes=lanes,
                             lane_probabilities=lane_probabilities,
                             lane_stds=lane_stds,
                             road_edges=road_edges,
                             road_edge_stds=road_edge_stds,
                             desire_state=desire_state)


def xyzt_message(points: List[ModelPoint], t_values: List[float]) -> DummyMessage:
    return DummyMessage(t=list(t_values),
                        x=[point.x for point in points],
                        y=[point.y for point in points],
                        z=[point.z for point in points])


def position_message(points: List[ModelPoint], stds: List[ModelPoint],
                     t_values: List[float]) -> DummyMessage:
    msg = xyzt_message(points, t_values)
    msg.xStd = [point.x for point in stds]
    msg.yStd = [point.y for point in stds]
    msg.zStd = [point.z for point in stds]
    return msg


def model_v2_message(snapshot: K230ModelSnapshot) -> DummyMessage:
    return DummyMessage(
        position=position_message(snapshot.plan, snapshot.plan_stds, snapshot.model_t),
        orientation=xyzt_message(snapshot.plan_orientations, snapshot.model_t),
        laneLines=[xyzt_message(lane, snapshot.lane_t) for lane in snapshot.lanes],
        laneLineProbs=list(snapshot.lane_probabilities),
        laneLineStds=list(snapshot.lane_stds),
        roadEdges=[xyzt_message(edge, snapshot.lane_t) for edge in snapshot.road_edges],
        roadEdgeStds=list(snapshot.road_edge_stds),
        meta=DummyMessage(desireState=list(snapshot.desire_state)),
    )


class PlannerSubMaster:
    def __init__(self):
        self.frame = -1
        self.logMonoTime = {"modelV2": 0}
        self.data = {}

    def update_data(self, car_state, controls_state, model_snapshot: K230ModelSnapshot):
        self.frame += 1
        self.logMonoTime["modelV2"] = model_snapshot.model_timestamp_ns
        self.data["carState"] = car_state
        self.data["controlsState"] = controls_state
        self.data["modelV2"] = model_v2_message(model_snapshot)

    def __getitem__(self, service: str):
        return self.data[service]


def decode_can_batch(payload: bytes) -> List[CanFrame]:
    if len(payload) < CAN_BATCH_HEADER.size:
        return []
    _timestamp_ns, valid, count, _dropped, _reserved = CAN_BATCH_HEADER.unpack_from(payload, 0)
    if not valid:
        return []
    frames: List[CanFrame] = []
    offset = CAN_BATCH_HEADER.size
    for _ in range(min(count, CAN_BATCH_MAX_FRAMES)):
        if offset + CAN_FRAME.size > len(payload):
            break
        address, src, _bus_time, data_len, flags, raw = CAN_FRAME.unpack_from(payload, offset)
        offset += CAN_FRAME.size
        if address > CAN_MAX_ADDRESS or src > 7 or data_len not in CAN_DLC_LENGTHS:
            continue
        # Match panda/openpilot receive semantics: TX echo/reject frames are
        # published on src + 128 / src + 192 so CarState/fingerprint ignores them.
        op_src = src
        if flags & 0x1:
            op_src += 128
        if flags & 0x2:
            op_src += 192
        frames.append(CanFrame(address=address, src=op_src, dat=raw[:data_len]))
    return frames


def encode_can_batch(frames: Iterable[CanFrame]) -> bytes:
    frame_list: List[CanFrame] = []
    dropped = 0
    for frame in frames:
        dat = bytes(frame.dat)
        if (frame.address < 0 or frame.address > CAN_MAX_ADDRESS or
                frame.src < 0 or frame.src > CAN_MAX_TX_BUS or
                len(dat) not in CAN_DLC_LENGTHS):
            dropped += 1
            continue
        if len(frame_list) >= CAN_BATCH_MAX_FRAMES:
            dropped += 1
            continue
        frame_list.append(CanFrame(address=frame.address, src=frame.src, dat=dat))

    payload = bytearray(CAN_BATCH_SIZE)
    CAN_BATCH_HEADER.pack_into(payload, 0, now_ns(), 1, len(frame_list), dropped, 0)
    offset = CAN_BATCH_HEADER.size
    for frame in frame_list:
        dat = bytes(frame.dat)
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


def decode_model_control(payload: bytes) -> Tuple[LateralTarget, LateralPlan]:
    if len(payload) >= LATERAL_TARGET.size + LATERAL_PLAN.size:
        target_offset = len(payload) - LATERAL_PLAN.size - LATERAL_TARGET.size
        plan_offset = len(payload) - LATERAL_PLAN.size
        valid, lookahead_x, target_y, heading, curvature = LATERAL_TARGET.unpack_from(payload, target_offset)
        unpacked = LATERAL_PLAN.unpack_from(payload, plan_offset)
        plan_valid = bool(unpacked[0])
        mpc_solution_valid = bool(unpacked[1])
        values = list(unpacked[2:2 + CONTROL_N * 4 + 1])
        psis = values[0:CONTROL_N]
        curvatures = values[CONTROL_N:CONTROL_N * 2]
        curvature_rates = values[CONTROL_N * 2:CONTROL_N * 3]
        d_path_points = values[CONTROL_N * 3:CONTROL_N * 4]
        output_scale = values[CONTROL_N * 4]
        return (
            LateralTarget(valid=bool(valid), lookahead_x=lookahead_x,
                          target_y=target_y, heading=heading, curvature=curvature),
            LateralPlan(valid=plan_valid, mpc_solution_valid=mpc_solution_valid,
                        psis=psis, curvatures=curvatures,
                        curvature_rates=curvature_rates,
                        d_path_points=d_path_points, output_scale=output_scale),
        )

    target = decode_lateral_target(payload)
    return target, LateralPlan(valid=target.valid, mpc_solution_valid=target.valid,
                               curvatures=[target.curvature] * CONTROL_N,
                               d_path_points=[target.target_y] * CONTROL_N)


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
        from selfdrive.controls.lib.drive_helpers import get_lag_adjusted_curvature
        from selfdrive.controls.lib.latcontrol_angle import LatControlAngle
        from selfdrive.controls.lib.latcontrol_atom import LatControlATOM
        from selfdrive.controls.lib.latcontrol_indi import LatControlINDI
        from selfdrive.controls.lib.latcontrol_lqr import LatControlLQR
        from selfdrive.controls.lib.latcontrol_pid import LatControlPID
        from selfdrive.controls.lib.latcontrol_torque import LatControlTorque
        from selfdrive.controls.lib.lateral_planner import LateralPlanner
        from selfdrive.controls.lib.vehicle_model import VehicleModel

        self.car = car
        self.log = log
        self.get_lag_adjusted_curvature = get_lag_adjusted_curvature
        self.CI_cls = CarInterface
        self.candidate = CAR.K7_HEV_YG
        self.fingerprint = fingerprint if fingerprint is not None else gen_empty_fingerprint()
        self.CP = CarInterface.get_params(self.candidate, self.fingerprint)
        self.CI = CarInterface(self.CP, CarController, CarState)
        self.enabled = env_enabled("K230_CONTROLD_ENABLED", True)
        self.VM = VehicleModel(self.CP)
        self.LaC = self.make_lateral_controller({
            "angle": LatControlAngle,
            "pid": LatControlPID,
            "indi": LatControlINDI,
            "lqr": LatControlLQR,
            "torque": LatControlTorque,
            "atom": LatControlATOM,
        })
        self.live_parameters = DummyMessage(stiffnessFactor=1.0,
                                            steerRatio=max(float(self.CP.steerRatio), 0.1),
                                            angleOffsetDeg=0.0,
                                            angleOffsetAverageDeg=0.0,
                                            roll=0.0)
        self.live_location_kalman = DummyMessage(
            angularVelocityCalibrated=DummyMessage(value=[0.0, 0.0, 0.0]))
        self.last_actuators = self.car.CarControl.Actuators.new_message()
        self.last_lac_output = 0.0
        self.last_control_curvature = 0.0
        self.last_planner_frame_id = None
        self.lateral_plan = LateralPlan()
        self.planner_sm = PlannerSubMaster()
        self.lateral_planner = LateralPlanner(self.CP)

    def make_lateral_controller(self, controller_classes):
        which = self.CP.lateralTuning.which()
        controller_cls = controller_classes.get(which)
        if controller_cls is None:
            raise RuntimeError(f"unsupported lateral tuning: {which}")
        return controller_cls(self.CP, self.CI)

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

    def planner_controls_state(self, car_state) -> DummyMessage:
        steer_angle_without_offset = math.radians(
            car_state.steeringAngleDeg - self.live_parameters.angleOffsetDeg)
        curvature = -self.VM.calc_curvature(steer_angle_without_offset,
                                            car_state.vEgo,
                                            self.live_parameters.roll)
        self.last_control_curvature = curvature
        lat_state = DummyMessage(output=self.last_lac_output)
        return DummyMessage(active=bool(self.enabled and car_state.cruiseState.enabled),
                            vCruise=0.0,
                            curvature=curvature,
                            pauseSpdLimit=False,
                            lateralControlState=DummyMessage(pidState=lat_state,
                                                             indiState=lat_state,
                                                             lqrState=lat_state,
                                                             torqueState=lat_state,
                                                             atomState=lat_state))

    def update_lateral_plan(self, car_state, model_snapshot: Optional[K230ModelSnapshot]):
        if model_snapshot is None or not model_snapshot.valid:
            self.lateral_plan = LateralPlan()
            self.last_planner_frame_id = None
            return
        if self.last_planner_frame_id == model_snapshot.frame_id:
            return

        controls_state = self.planner_controls_state(car_state)
        self.planner_sm.update_data(car_state, controls_state, model_snapshot)
        self.lateral_planner.update(self.planner_sm, self.CP)
        self.last_planner_frame_id = model_snapshot.frame_id

        x_sol = self.lateral_planner.lat_mpc.x_sol
        u_sol = self.lateral_planner.lat_mpc.u_sol
        self.lateral_plan = LateralPlan(
            valid=True,
            mpc_solution_valid=self.lateral_planner.solution_invalid_cnt < 2,
            psis=[float(x) for x in x_sol[0:CONTROL_N, 2]],
            curvatures=[float(x) for x in x_sol[0:CONTROL_N, 3]],
            curvature_rates=[float(x) for x in u_sol[0:CONTROL_N - 1]] + [0.0],
            d_path_points=[float(x) for x in self.lateral_planner.y_pts],
            output_scale=float(self.lateral_planner.DH.output_scale),
        )

    def update(self, frames: List[CanFrame], model_snapshot: Optional[K230ModelSnapshot]) -> List[CanFrame]:
        can_strings = self.can_strings(frames)
        cc = self.car.CarControl.new_message()
        car_state = self.CI.update(cc, can_strings)

        stiffness_factor = max(float(self.live_parameters.stiffnessFactor), 0.1)
        steer_ratio = max(float(self.live_parameters.steerRatio), 0.1)
        self.VM.update_params(stiffness_factor, steer_ratio)

        self.update_lateral_plan(car_state, model_snapshot)
        plan = self.lateral_plan

        cc.enabled = bool(self.enabled)
        cc.active = bool(self.enabled and plan.valid and
                         plan.mpc_solution_valid and car_state.cruiseState.enabled)

        desired_curvature, desired_curvature_rate = self.get_lag_adjusted_curvature(
            self.CP, car_state.vEgo, plan.psis, plan.curvatures, plan.curvature_rates)
        lat_active = bool(cc.active and
                          not car_state.steerFaultPermanent and
                          not (car_state.vEgo < self.CP.minSteerSpeed))
        steer, steering_angle_deg, _lac_log = self.LaC.update(
            lat_active, car_state, self.CP, self.VM, self.live_parameters,
            self.last_actuators, desired_curvature, desired_curvature_rate,
            self.live_location_kalman)
        self.last_lac_output = float(getattr(_lac_log, "output", steer))

        cc.actuators.steer = steer
        cc.actuators.steeringAngleDeg = steering_angle_deg
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
            if not (0 <= bus <= CAN_MAX_TX_BUS and
                    0 <= address <= CAN_MAX_ADDRESS and
                    len(dat) in CAN_DLC_LENGTHS):
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
        model_snapshot: Optional[K230ModelSnapshot] = None
        frames_in = 0
        frames_out = 0
        errors = 0
        last_log = time.monotonic()

        while not stop:
            last_model_seq, model_payload = model_sub.read_new(last_model_seq, 0)
            if model_payload:
                model_snapshot = decode_k230_model_state(model_payload)

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
                          f"latTune={controller.CP.lateralTuning.which()} "
                          f"sccBus={controller.CP.sccBus} "
                          f"mdpsBus={controller.CP.mdpsBus} sasBus={controller.CP.sasBus} "
                          f"safety={safety.safetyModel}:{safety.safetyParam} "
                          f"fingerprint_addrs={fingerprint_addr_count(fingerprint)}", flush=True)
                continue

            try:
                out_frames = controller.update(frames, model_snapshot)
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
                lateral_plan = controller.lateral_plan if controller is not None else LateralPlan()
                print(f"k230_controlsd: can_in={frames_in} sendcan={frames_out} "
                      f"errors={errors} model={int(model_snapshot.valid if model_snapshot else 0)} "
                      f"lat_plan={int(lateral_plan.valid)} "
                      f"mpc={int(lateral_plan.mpc_solution_valid)} "
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
