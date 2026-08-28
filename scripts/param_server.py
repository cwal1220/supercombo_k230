#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import stat
import tempfile
import threading
from pathlib import Path
from typing import Any, Callable, Dict

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel

if __package__:
    from .k230_display_control import DisplayBacklight
else:
    from k230_display_control import DisplayBacklight


CONTROLSD_NAME = "k230_controlsd"
RECORDD_NAME = "k230_recordd"
GROUP_ENV = {
    "steering": ("K230_STEERING_PARAMS", "steering.json"),
    "driving": ("K230_DRIVING_PARAMS", "driving.json"),
    "adaptive_cruise": (
        "K230_ADAPTIVE_CRUISE_PARAMS",
        "adaptive_cruise.json",
    ),
    "recording": ("K230_RECORDING_PARAMS", "recording.json"),
    "display": ("K230_DISPLAY_PARAMS", "display.json"),
}


def param_meta(
    label: str,
    section: str,
    unit: str,
    step: float,
    minimum: float,
    maximum: float,
    description: str,
    increase: str,
    decrease: str,
    *,
    quick: bool = False,
    quick_section: str | None = None,
    quick_order: int | None = None,
    control: str | None = None,
) -> Dict[str, Any]:
    metadata = {
        "label": label,
        "section": section,
        "unit": unit,
        "step": step,
        "min": minimum,
        "max": maximum,
        "description": description,
        "increase": increase,
        "decrease": decrease,
        "quick": quick,
        "quick_section": quick_section or section,
        "quick_order": quick_order,
    }
    if control is not None:
        metadata["control"] = control
    return metadata


PARAM_METADATA: Dict[str, Dict[str, Dict[str, Any]]] = {
    "steering": {
        "enabled": {
            "label": "자동 조향",
            "section": "기본 토크 제한",
            "description": "K7 조향 컨트롤러 전체를 켜거나 끕니다.",
            "increase": "켜면 모델 경로를 따라 조향 토크를 생성합니다.",
            "decrease": "끄면 조향 토크를 생성하지 않습니다.",
        },
        "steer_max": param_meta(
            "최대 조향 토크", "기본 토크 제한", "CAN torque", 5, 0, 384,
            "가변 토크 제한과 fault 회피에 사용하는 전체 토크 상한입니다.",
            "강한 커브에서 더 큰 토크를 허용합니다.",
            "최대 조향 힘이 줄어들어 급커브 추종이 약해질 수 있습니다.",
        ),
        "steer_delta_up": param_meta(
            "토크 증가 속도", "기본 토크 제한", "torque/frame", 1, 0, 20,
            "가변 제한에서 한 제어 주기마다 토크가 증가할 수 있는 양입니다.",
            "핸들이 목표 토크에 더 빠르게 도달합니다.",
            "조향 반응이 부드러워지지만 커브 진입이 늦을 수 있습니다.",
        ),
        "steer_delta_down": param_meta(
            "토크 감소 속도", "기본 토크 제한", "torque/frame", 1, 0, 30,
            "가변 제한에서 한 제어 주기마다 토크가 감소할 수 있는 양입니다.",
            "커브가 끝날 때 토크를 더 빨리 풉니다.",
            "토크가 더 천천히 풀려 조향이 부드럽지만 잔류할 수 있습니다.",
        ),
        "steer_driver_allowance": param_meta(
            "운전자 토크 여유", "운전자 개입", "MDPS raw", 5, 0, 300,
            "운전자 조향 토크 제한 계산에서 허용하는 기본 여유입니다.",
            "운전자 입력 중에도 자동 조향 토크가 더 오래 유지됩니다.",
            "운전자 입력에 자동 조향이 더 빨리 양보합니다.",
        ),
        "steer_driver_multiplier": param_meta(
            "운전자 토크 배율", "운전자 개입", "배", 1, 0, 10,
            "운전자 토크가 허용 자동 조향 토크에 미치는 배율입니다.",
            "운전자 입력이 토크 제한에 더 강하게 반영됩니다.",
            "운전자 입력에 따른 토크 제한 변화가 작아집니다.",
        ),
        "steer_driver_factor": param_meta(
            "운전자 토크 계수", "운전자 개입", "배", 1, 0, 5,
            "MDPS에서 읽은 운전자 토크에 적용하는 계수입니다.",
            "같은 핸들 입력을 더 큰 운전자 개입으로 판단합니다.",
            "같은 핸들 입력을 더 작게 판단합니다.",
        ),
        "steering_pressed_threshold": param_meta(
            "운전자 조향 감지값", "운전자 개입", "MDPS raw", 10, 0, 500,
            "PID 적분을 멈추는 운전자 조향 토크 기준입니다.",
            "더 강하게 핸들을 잡아야 운전자 개입으로 판단합니다.",
            "작은 핸들 입력도 더 빨리 운전자 개입으로 판단합니다.",
        ),
        "torque_max_lat_accel_raw": param_meta(
            "토크 기준 횡가속도", "토크 컨트롤러", "0.1 m/s²", 1, 1, 80,
            "kp/kf/ki가 전부 이 값에서 파생되는 횡방향 주 이득입니다. "
            "fit_lateral_params.py fit으로 실측한 값을 넣습니다.",
            "전체 이득이 약해져 추종이 느려지고 언더스티어가 늘어납니다.",
            "전체 이득이 강해져 추종이 빨라지고 포화 여유가 줄어듭니다.",
            quick=True, quick_section="조향 반응", quick_order=10,
        ),
        "torque_kp_raw": param_meta(
            "비례 이득 Kp", "토크 컨트롤러", "raw", 1, 0, 100,
            "현재 횡가속도 오차에 바로 반응하는 비례 이득입니다.",
            "경로 오차를 더 빠르고 강하게 보정하지만 흔들림이 늘 수 있습니다.",
            "반응이 부드러워지지만 경로 오차 회복이 느려질 수 있습니다.",
        ),
        "torque_kf_raw": param_meta(
            "선행 보상 Kf", "토크 컨트롤러", "raw", 1, 0, 100,
            "목표 횡가속도에 미리 더하는 feed-forward 이득입니다.",
            "커브에서 기본 조향 토크가 커집니다.",
            "커브에서 선행 조향 토크가 작아집니다.",
        ),
        "torque_ki_raw": param_meta(
            "적분 이득 Ki", "토크 컨트롤러", "raw", 1, 0, 100,
            "지속되는 횡가속도 오차를 누적해 없애는 적분 이득입니다.",
            "지속 오차를 빨리 없애지만 오버슈트가 늘 수 있습니다.",
            "누적 보정이 느려져 일정한 편향이 오래 남을 수 있습니다.",
        ),
        "torque_friction_raw": param_meta(
            "조향 마찰 보상", "토크 컨트롤러", "0.001 m/s²", 5, 0, 300,
            "조향계 마찰을 넘기 위해 방향 전환 시 더하는 보상입니다.",
            "작은 커브에도 핸들이 더 즉각 움직이지만 좌우 튐이 생길 수 있습니다.",
            "미세 조향이 부드러워지지만 dead zone이 커질 수 있습니다.",
        ),
        "torque_use_angle": {
            "label": "조향각 기반 곡률",
            "section": "토크 컨트롤러",
            "description": "실제 곡률 계산에 조향각 센서를 우선 사용합니다.",
            "increase": "켜면 조향각 기반 곡률을 사용합니다.",
            "decrease": "끄면 유효한 ESP yaw-rate와 속도별로 혼합합니다.",
        },
        "torque_output_sign": param_meta(
            "토크 출력 방향", "고정 차량 설정", "부호", 2, -1, 1,
            "K7 YG HEV의 조향 토크 방향입니다. 정상값은 -1입니다.",
            "1로 바뀌면 조향 방향이 반전됩니다.",
            "-1로 바뀌면 K7 기준 정상 조향 방향이 됩니다.",
        ),
        "smooth_steer_method": param_meta(
            "Smooth steer 모드", "Smooth steer", "mode", 1, 0, 1,
            "운전자 입력과 큰 조향각에서 토크를 줄이는 방식을 선택합니다.",
            "1이면 각도 기반 smooth steer를 사용합니다.",
            "0이면 일반 운전자 토크 fade를 사용합니다.",
        ),
        "smooth_max_steering_angle_deg": param_meta(
            "Smooth steer 시작 각도", "Smooth steer", "°", 5, 0, 180,
            "Smooth steer가 큰 조향각으로 판단하는 기준입니다.",
            "더 큰 핸들 각도까지 토크 감소를 늦춥니다.",
            "더 작은 핸들 각도부터 토크를 줄입니다.",
        ),
        "smooth_max_driver_angle_wait": param_meta(
            "큰 각도 운전자 감쇠", "Smooth steer", "ratio/frame", 0.001, 0, 1,
            "큰 조향각에서 운전자 입력이 있을 때 프레임마다 줄이는 토크 비율입니다.",
            "운전자 개입 시 토크를 더 빨리 줄입니다.",
            "운전자 개입 시 토크를 더 천천히 줄입니다.",
        ),
        "smooth_max_steer_angle_wait": param_meta(
            "큰 각도 자동 감쇠", "Smooth steer", "ratio/frame", 0.001, 0, 1,
            "큰 조향각에서 운전자 입력이 없을 때 프레임마다 줄이는 토크 비율입니다.",
            "큰 조향각의 자동 토크를 더 빨리 줄입니다.",
            "큰 조향각의 자동 토크를 더 오래 유지합니다.",
        ),
        "smooth_driver_angle_wait": param_meta(
            "일반 각도 운전자 감쇠", "Smooth steer", "ratio/frame", 0.001, 0, 1,
            "일반 조향각에서 운전자 입력이 있을 때 프레임마다 줄이는 토크 비율입니다.",
            "운전자 입력에 자동 조향이 더 빨리 양보합니다.",
            "자동 조향 토크를 더 오래 유지합니다.",
        ),
        "steer_ratio": param_meta(
            "조향비", "차량 모델", "ratio", 0.1, 8, 25,
            "핸들 조향각과 전륜 조향각 사이의 차량 조향비입니다.",
            "같은 핸들 각도를 더 작은 전륜 조향으로 추정합니다.",
            "같은 핸들 각도를 더 큰 전륜 조향으로 추정합니다.",
        ),
        "tire_stiffness_factor": param_meta(
            "타이어 횡강성", "차량 모델", "배", 0.05, 0.2, 2,
            "차량 모델의 기준 타이어 횡강성에 곱하는 보정값입니다.",
            "타이어가 횡력에 더 단단하게 반응한다고 계산합니다.",
            "타이어가 더 유연하게 반응한다고 계산합니다.",
        ),
        "steer_actuator_delay": param_meta(
            "조향 반응 지연", "조향 반응", "초", 0.01, 0.01, 1,
            "현재 조향 명령이 차량에 반영되기까지의 예측 지연입니다.",
            "경로를 더 앞에서 읽어 커브 진입을 선행하지만 과하면 오버슈트할 수 있습니다.",
            "조향 선행량이 줄어 커브 반응이 늦어질 수 있습니다.",
            quick=True, quick_section="조향 반응", quick_order=20,
        ),
        "max_steering_angle_deg": param_meta(
            "최대 자동 조향각", "LKAS fault 보호", "°", 5, 0, 360,
            "LKAS fault 회피가 꺼졌을 때 적용하는 절대 조향각입니다. 0은 제한을 끕니다.",
            "더 큰 핸들 각도까지 자동 조향을 허용합니다.",
            "더 이른 조향각에서 자동 조향을 제한합니다.",
        ),
        "avoid_lkas_fault_enabled": {
            "label": "LKAS fault 회피",
            "section": "LKAS fault 보호",
            "description": "큰 조향각이 지속될 때 steer request를 잠시 끊어 fault를 회피합니다.",
            "increase": "켜면 K7용 request pulse 회피 로직을 사용합니다.",
            "decrease": "끄면 큰 조향각에서도 request를 계속 유지합니다.",
        },
        "avoid_lkas_fault_max_angle_deg": param_meta(
            "Fault 감시 조향각", "LKAS fault 보호", "°", 1, 1, 180,
            "LKAS fault 회피 카운터를 시작하는 절대 조향각입니다.",
            "더 큰 핸들 각도에서 회피 동작을 시작합니다.",
            "더 작은 핸들 각도부터 회피 동작을 준비합니다.",
        ),
        "avoid_lkas_fault_max_frames": param_meta(
            "Fault 허용 프레임", "LKAS fault 보호", "frame", 1, 0, 300,
            "큰 조향각에서 request를 유지할 수 있는 최대 100 Hz 프레임 수입니다.",
            "request를 더 오래 유지한 뒤 잠시 끊습니다.",
            "request를 더 일찍 끊어 fault를 회피합니다.",
        ),
        "no_smart_mdps": {
            "label": "비 Smart MDPS 모드",
            "section": "고정 차량 설정",
            "description": "최소 조향 속도 아래에서 제어 전체를 차단하는 호환 모드입니다.",
            "increase": "켜면 저속에서 제어를 차단합니다.",
            "decrease": "끄면 K7 Smart MDPS 동작을 유지합니다.",
        },
        "turn_steering_disable": {
            "label": "저속 방향지시등 조향 차단",
            "section": "운전자 개입",
            "description": "저속에서 방향지시등을 켰을 때 자동 조향을 잠시 차단합니다.",
            "increase": "켜면 설정 속도 아래에서 차선 변경 조향을 운전자에게 넘깁니다.",
            "decrease": "끄면 방향지시등 중에도 desire 경로를 따라 자동 조향합니다.",
        },
        "live_bank_compensation": {
            "label": "실시간 편경사 보정",
            "section": "차량 중심 보정",
            "description": "ESP12 실측 횡가속으로 추정한 도로 편경사를 FF에서 보정합니다.",
            "increase": "켜면 커브별 편경사까지 실시간 보정합니다.",
            "decrease": "끄면 상수 offset만 사용합니다.",
        },
        "torque_lat_accel_offset": param_meta(
            "횡가속 편향 보정", "차량 중심 보정", "m/s²", 0.01, -1.0, 1.0,
            "장착 롤 오차 등이 만드는 상수 횡가속 편향을 feed-forward에서 "
            "뺍니다. fit_lateral_params.py fit의 latAccelOffset을 그대로 넣습니다.",
            "차가 오른쪽으로 쏠릴 때 키우는 방향입니다.",
            "차가 왼쪽으로 쏠릴 때 줄이는 방향입니다.",
            quick=True, quick_section="주행 위치", quick_order=20,
        ),
        "angle_offset_deg": param_meta(
            "직진 조향각 오프셋", "차량 중심 보정", "°", 0.1, -10, 10,
            "직진 상태의 조향각 센서 편차를 실제 곡률 계산 전에 뺍니다.",
            "현재 센서 각도를 더 작게 보정합니다.",
            "현재 센서 각도를 더 크게 보정합니다.",
        ),
        "roll_rad": param_meta(
            "차량 Roll 보정", "차량 모델", "rad", 0.001, -0.2, 0.2,
            "도로 기울기 또는 차량 roll이 만드는 횡가속도를 보정합니다.",
            "우측 방향 중력 보정량이 커집니다.",
            "좌측 방향 중력 보정량이 커집니다.",
        ),
        "mass_kg": param_meta(
            "차량 질량", "차량 모델", "kg", 10, 1000, 2600,
            "차량 모델과 타이어 횡강성 계산에 사용하는 질량입니다.",
            "차량이 더 무겁다고 계산합니다.",
            "차량이 더 가볍다고 계산합니다.",
        ),
        "wheelbase_m": param_meta(
            "축거", "차량 모델", "m", 0.01, 2, 3.5,
            "전륜과 후륜 사이 거리입니다.",
            "같은 곡률에 더 큰 조향각이 필요하다고 계산합니다.",
            "같은 곡률에 더 작은 조향각이 필요하다고 계산합니다.",
        ),
        "center_to_front_ratio": param_meta(
            "전축 무게중심 비율", "차량 모델", "ratio", 0.01, 0.2, 0.7,
            "무게중심에서 전축까지 거리를 축거 비율로 나타냅니다.",
            "무게중심을 후방 쪽으로 계산합니다.",
            "무게중심을 전방 쪽으로 계산합니다.",
        ),
        "steer_ratio_rear": param_meta(
            "후륜 조향비", "고정 차량 설정", "ratio", 0.01, -0.5, 0.5,
            "후륜 조향 차량의 곡률 보정값입니다. K7 정상값은 0입니다.",
            "후륜 조향의 양의 보정량이 커집니다.",
            "후륜 조향의 음의 보정량이 커집니다.",
        ),
        "camera_offset_m": param_meta(
            "카메라 좌우 위치", "차량 중심 보정", "m", 0.01, -1, 1,
            "차량 중심에 대한 카메라 위치를 차선 검출선에 보정합니다.",
            "목표 차선 중심이 차량 기준 오른쪽으로 이동합니다.",
            "목표 차선 중심이 차량 기준 왼쪽으로 이동합니다.",
            quick=True, quick_section="주행 위치", quick_order=30
        ),
        "path_offset_m": param_meta(
            "주행 경로 좌우 보정", "차량 중심 보정", "m", 0.01, -1, 1,
            "최종 모델 주행 경로 전체를 좌우로 평행 이동합니다.",
            "목표 주행 위치가 차량 기준 오른쪽으로 이동합니다.",
            "목표 주행 위치가 차량 기준 왼쪽으로 이동합니다.",
            quick=True, quick_section="주행 위치", quick_order=10,
        ),
        "min_steer_speed_mps": param_meta(
            "최소 자동 조향 속도", "기본 토크 제한", "m/s", 0.1, 0, 5,
            "이 속도 미만에서 토크 컨트롤러 출력을 0으로 만듭니다.",
            "자동 조향이 시작되는 최소 속도가 높아집니다.",
            "더 낮은 속도에서도 자동 조향을 허용합니다.",
        ),
    },
    "driving": {
        "model_timeout_ms": param_meta(
            "모델 경로 유효 시간", "데이터 상태", "ms", 50, 50, 2000,
            "마지막 모델 경로를 유효하다고 인정하는 최대 시간입니다.",
            "모델 갱신이 늦어도 기존 경로를 더 오래 사용합니다.",
            "모델 정지 시 더 빨리 조향을 차단합니다.",
        ),
        "vehicle_state_timeout_ms": param_meta(
            "차량 상태 유효 시간", "데이터 상태", "ms", 50, 50, 2000,
            "CAN 차량 상태와 yaw-rate를 유효하다고 인정하는 최대 시간입니다.",
            "CAN 지연을 더 오래 허용하지만 오래된 상태를 쓸 수 있습니다.",
            "CAN 갱신이 멈추면 더 빨리 제어를 차단합니다.",
        ),
        "inactive_release_ms": param_meta(
            "Disengage 토크 해제 시간", "상태와 CAN", "ms", 100, 0, 5000,
            "Disengage 후 순정 LKAS에 넘기기 전 0 토크 프레임을 유지하는 시간입니다.",
            "0 토크 handoff를 더 오래 유지합니다.",
            "순정 LKAS로 더 빨리 제어권을 넘깁니다.",
        ),
        "mdps_speed_spoof_kph": param_meta(
            "MDPS 위조 속도", "고정 차량 설정", "km/h", 1, 30, 100,
            "K7 MDPS가 저속에서도 LKAS 조향을 허용하도록 보내는 속도입니다.",
            "MDPS에 더 높은 차량 속도로 보냅니다.",
            "MDPS에 더 낮은 차량 속도로 보냅니다. K7 기준은 60 km/h입니다.",
        ),
        "lane_change_min_speed_kph": param_meta(
            "저속 보호 기준", "운전자 개입", "km/h", 1, 0, 80,
            "운전자 토크 보호와 저속 방향지시등 조향 차단의 속도 기준입니다.",
            "보호 로직이 적용되는 속도 구간이 넓어집니다.",
            "보호 로직이 적용되는 속도 구간이 줄어듭니다.",
        ),
        "manual_steer_disable_frames": param_meta(
            "수동 조향 유지 시간", "운전자 개입", "frame", 10, 0, 500,
            "저속 방향지시등 조작 후 자동 조향을 차단하는 100 Hz 프레임 수입니다.",
            "운전자에게 조향을 넘기는 시간이 길어집니다.",
            "자동 조향으로 더 빨리 복귀합니다.",
        ),
        "driver_torque_threshold": param_meta(
            "저속 운전자 토크 기준", "운전자 개입", "MDPS raw", 10, 0, 500,
            "저속에서 자동 토크를 줄이기 시작하는 운전자 조향 토크입니다.",
            "더 강한 운전자 입력에서만 자동 토크를 줄입니다.",
            "작은 운전자 입력에도 자동 토크를 더 빨리 줄입니다.",
        ),
    },
    "adaptive_cruise": {
        "enabled": {
            "label": "비전 크루즈",
            "section": "동작",
            "description": "비전 선행차를 기준으로 순정 크루즈의 SET-/RES+ 버튼을 자동 조절합니다.",
            "increase": "켜면 최초 SET 속도를 상한으로 비전 기반 속도 조절을 시작합니다.",
            "decrease": "끄면 자동 버튼 명령을 중지하고 현재 순정 크루즈 설정에 개입하지 않습니다.",
            "quick": True,
            "quick_section": "동작",
            "quick_order": 10,
        },
        "following_time_s": param_meta(
            "주행 차간시간", "차간 거리", "초", 0.1, 0.8, 4.0,
            "현재 속도에 곱해 선행차와 유지할 동적 거리를 계산합니다.",
            "속도에 비례한 차간거리가 늘어 더 일찍 감속합니다.",
            "차간거리가 짧아지고 선행차에 더 가깝게 주행합니다.",
            quick=True, quick_section="차간 거리", quick_order=20,
        ),
        "standstill_gap_m": param_meta(
            "기본 차간거리", "차간 거리", "m", 0.5, 2.0, 20.0,
            "속도와 무관하게 목표 차간거리에 더하는 기본 거리입니다.",
            "모든 속도에서 선행차와 더 멀리 떨어집니다.",
            "모든 속도에서 선행차와 더 가까워집니다.",
            quick=True, quick_section="차간 거리", quick_order=30,
        ),
        "gap_correction_gain": param_meta(
            "거리 오차 반응", "속도 반응", "gain", 0.05, 0.05, 1.0,
            "실제 거리와 목표 거리의 차이를 목표 속도 보정으로 바꾸는 비율입니다.",
            "차간거리 변화에 더 빠르고 크게 반응하지만 속도 변동이 늘 수 있습니다.",
            "반응이 부드러워지지만 가까워지는 차량에 늦게 대응할 수 있습니다.",
        ),
        "max_slowdown_correction_mps": param_meta(
            "최대 감속 보정", "속도 반응", "m/s", 0.5, 0.5, 10.0,
            "선행차가 가깝거나 느릴 때 목표 속도를 낮추는 최대 보정량입니다.",
            "더 낮은 크루즈 설정을 요청해 감속 반응이 적극적이 됩니다.",
            "목표 속도 감소 폭이 작아져 감속 반응이 완만해집니다.",
        ),
        "max_speedup_correction_mps": param_meta(
            "최대 가속 보정", "속도 반응", "m/s", 0.5, 0.0, 5.0,
            "차간거리가 충분할 때 선행차 속도보다 높게 잡을 수 있는 최대 보정량입니다.",
            "저장된 최대 속도로 더 빠르게 복귀할 수 있습니다.",
            "속도 복귀가 보수적이고 느려집니다.",
        ),
        "deceleration_rate_kph_per_s": param_meta(
            "실측 감속 응답", "속도 반응", "km/h/s", 0.1, 0.5, 5.0,
            "SET- 명령 뒤 실제 차량 속도가 1초 동안 감소하는 실측값입니다. 다음 감속 명령 간격과 예상 차간거리 계산에 사용합니다.",
            "차량이 더 빨리 감속한다고 판단해 다음 SET-를 더 일찍 허용합니다.",
            "SET- 효과를 더 오래 기다리고 선행차와 가까워질 거리를 더 멀리 예측합니다.",
        ),
        "lead_restore_delay_s": param_meta(
            "선행차 소실 후 복귀", "복귀 동작", "초", 0.5, 1.0, 10.0,
            "선행차가 사라진 뒤 저장된 최대 속도로 복귀하기 전 기다리는 시간입니다.",
            "비전 검출이 끊겼을 때 현재 속도를 더 오래 유지합니다.",
            "선행차가 사라지면 최대 속도로 더 빨리 복귀합니다.",
        ),
        "command_interval_s": param_meta(
            "속도 변경 명령 간격", "버튼 송신", "초", 0.1, 0.5, 5.0,
            "연속 SET-/RES+ 버튼 펄스를 시작할 수 있는 최소 시간 간격입니다.",
            "크루즈 설정 속도가 더 천천히 변합니다.",
            "크루즈 설정 속도가 더 빠르게 변하지만 잦은 버튼 명령이 발생합니다.",
        ),
        "lead_probability_threshold": param_meta(
            "선행차 인식 확률", "비전 판정", "0~1", 0.05, 0.2, 0.99,
            "모델 선행차를 속도 제어에 사용하기 위한 최소 확률입니다. 값은 0~1 비율입니다.",
            "확실한 선행차만 사용해 오검출은 줄지만 검출이 늦을 수 있습니다.",
            "선행차를 더 빨리 받아들이지만 오검출 가능성이 커집니다.",
        ),
        "lead_hold_s": param_meta(
            "선행차 유지 시간", "비전 판정", "초", 0.1, 0.1, 2.0,
            "일시적으로 비전 lead가 끊겨도 마지막 선행차를 유효하게 유지하는 시간입니다.",
            "짧은 검출 누락에 덜 흔들리지만 오래된 lead를 더 오래 사용합니다.",
            "오래된 lead를 빨리 버리지만 검출 흔들림에 민감해집니다.",
        ),
        "button_pulse_frames": param_meta(
            "버튼 펄스 길이", "버튼 송신", "100 Hz frame", 1, 1, 10,
            "한 번의 SET-/RES+ 조작을 차량에 전달할 연속 CAN 프레임 수입니다.",
            "차량이 버튼을 인식하기 쉬워지지만 길게 누른 것으로 해석될 수 있습니다.",
            "펄스가 짧아지며 너무 작으면 차량이 명령을 놓칠 수 있습니다.",
        ),
    },
    "recording": {
        "enabled": {
            "label": "주행 데이터 기록",
            "section": "기록",
            "description": "모델이 실제 사용한 영상과 CAN 송수신, 모델·제어 상태를 함께 저장합니다.",
            "increase": "켜면 하드웨어 H.265 인코더로 기록을 시작합니다.",
            "decrease": "끄면 현재 기록을 안전하게 닫고 인코더를 유휴 상태로 둡니다.",
            "quick": True,
            "quick_section": "기록",
            "quick_order": 10,
        },
    },
    "display": {
        "enabled": {
            "label": "디스플레이 전원",
            "section": "백라이트",
            "description": "LCD 영상 출력은 유지한 채 백라이트만 켜거나 끕니다.",
            "increase": "GPIO25를 High로 설정하거나 저장된 밝기의 PWM을 다시 켭니다.",
            "decrease": "GPIO25를 Low로 설정해 백라이트를 완전히 끕니다.",
            "quick": True,
            "quick_section": "백라이트",
            "quick_order": 10,
        },
        "brightness_percent": param_meta(
            "화면 밝기", "백라이트", "%", 1, 1, 100,
            "패널의 저·고밝기 구간을 나누어 실측 보정한 20 kHz PWM입니다. 전원을 꺼도 이 값은 유지됩니다.",
            "화면이 밝아집니다. 100%에서는 GPIO High를 사용합니다.",
            "화면이 어두워집니다. 완전히 끄려면 전원 스위치를 사용합니다.",
            quick=True, quick_section="백라이트", quick_order=20,
            control="slider",
        ),
    },
}


def configured_paths() -> Dict[str, Path]:
    params_dir = Path(os.environ.get("K230_PARAMS_DIR", "params"))
    return {
        group: Path(os.environ.get(env_name, params_dir / filename))
        for group, (env_name, filename) in GROUP_ENV.items()
    }


def configured_default_paths() -> Dict[str, Path]:
    params_dir = Path(os.environ.get("K230_PARAM_DEFAULTS_DIR", "params.defaults"))
    return {
        group: params_dir / filename
        for group, (_, filename) in GROUP_ENV.items()
    }


def find_process_pids(process_name: str) -> list[int]:
    pids = []
    proc = Path("/proc")
    if not proc.is_dir():
        return pids
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv0 = (entry / "cmdline").read_bytes().split(b"\0", 1)[0]
            if Path(os.fsdecode(argv0)).name == process_name:
                pids.append(int(entry.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
            continue
    return sorted(pids)


def find_controlsd_pids() -> list[int]:
    return find_process_pids(CONTROLSD_NAME)


def find_recordd_pids() -> list[int]:
    return find_process_pids(RECORDD_NAME)


def notify_controlsd() -> list[int]:
    notified = []
    for pid in find_controlsd_pids():
        try:
            os.kill(pid, signal.SIGHUP)
            notified.append(pid)
        except (PermissionError, ProcessLookupError):
            continue
    return notified


class ParamStore:
    def __init__(
        self,
        paths: Dict[str, Path] | None = None,
        notifier: Callable[[], list[int]] = notify_controlsd,
        default_paths: Dict[str, Path] | None = None,
        display_controller: DisplayBacklight | None = None,
    ):
        self.paths = paths or configured_paths()
        self.notifier = notifier
        self.lock = threading.Lock()
        self.default_paths = (
            default_paths
            if default_paths is not None
            else (configured_default_paths() if paths is None else {})
        )
        self.display_controller = display_controller
        self._merge_missing_defaults()
        if self.display_controller is not None and "display" in self.paths:
            try:
                self.display_controller.apply(self.read_group("display"))
            except (RuntimeError, ValueError):
                pass

    def _merge_missing_defaults(self) -> None:
        for group, default_path in self.default_paths.items():
            runtime_path = self.paths.get(group)
            if runtime_path is None or not default_path.is_file():
                continue
            try:
                defaults = json.loads(default_path.read_text(encoding="utf-8"))
                runtime = (
                    json.loads(runtime_path.read_text(encoding="utf-8"))
                    if runtime_path.is_file()
                    else {}
                )
            except json.JSONDecodeError:
                continue
            if not isinstance(defaults, dict) or not isinstance(runtime, dict):
                continue
            merged = {**defaults, **runtime}
            if merged != runtime:
                self._atomic_write(runtime_path, merged)

    def read_group(self, group: str) -> Dict[str, Any]:
        path = self._path(group)
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise ValueError(f"parameter file not found: {path}") from exc
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid JSON in {path}: {exc}") from exc
        if not isinstance(data, dict):
            raise ValueError(f"parameter document must be an object: {path}")
        return data

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            documents = {group: self.read_group(group) for group in self.paths}
        return {
            "params": documents,
            "metadata": PARAM_METADATA,
            "paths": {group: str(path) for group, path in self.paths.items()},
            "controlsd_pids": find_controlsd_pids(),
            "recordd_pids": find_recordd_pids(),
            "display_status": self._display_status(),
        }

    def update(self, group: str, values: Dict[str, Any]) -> Dict[str, Any]:
        if not values:
            raise ValueError("no parameter values supplied")
        with self.lock:
            document = self.read_group(group)
            unknown = sorted(set(values) - set(document))
            if unknown:
                raise KeyError(", ".join(unknown))
            document.update(values)
            if group == "display":
                if self.display_controller is None:
                    raise ValueError("display backlight control is unavailable")
                self.display_controller.apply(document)
            self._atomic_write(self._path(group), document)
        notified = [] if group in ("recording", "display") else self.notifier()
        return {
            "group": group,
            "values": values,
            "params": document,
            "notified_pids": notified,
            "controlsd_pids": find_controlsd_pids(),
            "recordd_pids": find_recordd_pids(),
            "display_status": self._display_status(),
        }

    def _display_status(self) -> Dict[str, Any]:
        if self.display_controller is None:
            return {
                "available": False,
                "enabled": False,
                "brightness_percent": 0,
                "mode": "unavailable",
                "error": "display backlight control is unavailable",
            }
        return self.display_controller.status()

    def _path(self, group: str) -> Path:
        if group not in self.paths:
            raise KeyError(group)
        return self.paths[group]

    @staticmethod
    def _atomic_write(path: Path, document: Dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        mode = stat.S_IMODE(path.stat().st_mode) if path.exists() else 0o664
        fd, temporary_name = tempfile.mkstemp(
            prefix=path.name + ".", suffix=".tmp", dir=path.parent
        )
        temporary = Path(temporary_name)
        try:
            os.fchmod(fd, mode)
            with os.fdopen(fd, "w", encoding="utf-8") as file:
                json.dump(document, file, ensure_ascii=False, indent=2)
                file.write("\n")
                file.flush()
                os.fsync(file.fileno())
            os.replace(temporary, path)
            directory_fd = os.open(path.parent, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if temporary.exists():
                temporary.unlink()


class ParamPatch(BaseModel):
    values: Dict[str, Any]


HTML = """<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <title>K7 실시간 튜닝</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #101214;
      color: #f4f6f7;
      --surface: #191c1f;
      --surface-raised: #202428;
      --line: #353b40;
      --muted: #a5adb4;
      --accent: #58a6e7;
      --good: #4bc78d;
      --warn: #efb85b;
      --bad: #ed7474;
    }
    * { box-sizing: border-box; }
    body { margin: 0; min-width: 320px; background: #101214; }
    button, input { font: inherit; }
    button { touch-action: manipulation; }
    .app-header {
      position: sticky; top: 0; z-index: 5;
      display: flex; align-items: center; gap: 14px;
      min-height: 64px; padding: 10px max(16px, env(safe-area-inset-right)) 10px max(16px, env(safe-area-inset-left));
      background: #171a1d; border-bottom: 1px solid var(--line);
    }
    .brand { min-width: 0; }
    h1 { margin: 0; font-size: 19px; line-height: 1.2; font-weight: 750; letter-spacing: 0; }
    .connection {
      display: flex; align-items: center; gap: 7px;
      color: var(--muted); font-size: 13px; white-space: nowrap;
    }
    .dot { width: 9px; height: 9px; flex: 0 0 auto; border-radius: 50%; background: var(--bad); }
    .dot.online { background: var(--good); }
    .icon-button {
      width: 44px; height: 44px; margin-left: auto; padding: 0;
      border: 1px solid #4a5157; border-radius: 6px;
      background: #262b2f; color: #fff; font-size: 24px; line-height: 1;
      cursor: pointer;
    }
    .icon-button:hover { background: #31373c; }
    .group-tabs {
      position: sticky; top: 64px; z-index: 4;
      display: grid; grid-template-columns: repeat(5, minmax(0, 1fr));
      padding: 0 max(16px, env(safe-area-inset-right)) 0 max(16px, env(safe-area-inset-left));
      background: #171a1d; border-bottom: 1px solid var(--line);
    }
    .group-tab {
      min-height: 50px; border: 0; border-bottom: 3px solid transparent;
      border-radius: 0; background: transparent; color: var(--muted);
      padding: 0 4px; cursor: pointer; font-size: 13px; font-weight: 750;
    }
    .group-tab.active { color: #fff; border-bottom-color: var(--accent); }
    main {
      width: min(1100px, 100%); margin: 0 auto;
      padding: 16px max(16px, env(safe-area-inset-right)) max(40px, env(safe-area-inset-bottom)) max(16px, env(safe-area-inset-left));
    }
    .view-bar {
      display: flex; align-items: center; gap: 12px; justify-content: space-between;
      margin-bottom: 14px;
    }
    .view-tabs {
      display: inline-grid; grid-template-columns: 1fr 1fr;
      border: 1px solid #454c52; border-radius: 7px; overflow: hidden;
    }
    .view-tab {
      min-height: 40px; padding: 7px 14px; border: 0; border-right: 1px solid #454c52;
      border-radius: 0; background: #1a1e21; color: var(--muted);
      cursor: pointer; font-weight: 700;
    }
    .view-tab:last-child { border-right: 0; }
    .view-tab.active { background: #2b343a; color: #fff; }
    .count { color: var(--muted); font-size: 13px; }
    .group-note {
      display: none; margin-bottom: 14px; padding: 11px 12px;
      border: 1px solid #6a5630; border-radius: 6px;
      background: #241f16; color: #e6c985; font-size: 13px; line-height: 1.45;
    }
    .group-note.visible { display: block; }
    #message {
      display: none; margin-bottom: 14px; padding: 10px 12px;
      border: 1px solid #366249; border-radius: 6px;
      background: #17261f; color: #79daa9; font-size: 13px;
    }
    #message.visible { display: block; }
    #message.failed { border-color: #743f3f; background: #2a1919; color: #f08b8b; }
    .section { margin: 0 0 24px; }
    .section-title {
      display: flex; align-items: baseline; gap: 9px;
      margin: 0 0 9px; color: #e5e9ec; font-size: 15px; font-weight: 750;
    }
    .section-title span { color: #7f8991; font-size: 12px; font-weight: 600; }
    .param-list {
      display: grid; grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    .param-card {
      min-width: 0; padding: 14px;
      border: 1px solid var(--line); border-radius: 7px;
      background: var(--surface);
    }
    .param-card.busy { opacity: 0.66; }
    .param-card.saved { border-color: #397659; }
    .param-card.error { border-color: #8a4949; }
    .param-head { display: flex; align-items: flex-start; gap: 12px; justify-content: space-between; }
    .param-title { margin: 0; font-size: 16px; line-height: 1.3; font-weight: 750; letter-spacing: 0; }
    .param-unit { color: var(--warn); font-size: 12px; white-space: nowrap; }
    .param-key {
      margin-top: 3px; color: #737e86;
      font: 11px ui-monospace, SFMono-Regular, Menlo, monospace;
      overflow-wrap: anywhere;
    }
    .description { min-height: 40px; margin: 11px 0 9px; color: #c5cbd0; font-size: 13px; line-height: 1.5; }
    .effects {
      display: grid; grid-template-columns: 1fr 1fr; gap: 8px;
      margin-bottom: 13px;
    }
    .effect {
      padding: 8px 9px; border-left: 3px solid #576068;
      background: #15181a; color: #aeb6bc; font-size: 12px; line-height: 1.45;
    }
    .effect.up { border-left-color: var(--warn); }
    .effect.down { border-left-color: var(--accent); }
    .effect b { display: block; margin-bottom: 2px; color: #e5e9ec; font-size: 11px; }
    .number-control {
      display: grid; grid-template-columns: 54px minmax(100px, 1fr) 54px;
      gap: 8px; height: 50px;
    }
    .adjust-button {
      border: 1px solid #515960; border-radius: 6px;
      background: #293036; color: #fff; cursor: pointer;
      font-size: 28px; font-weight: 500; line-height: 1;
    }
    .adjust-button:hover { background: #363e44; }
    .value-wrap { position: relative; min-width: 0; }
    .value-input {
      width: 100%; height: 50px; padding: 5px 9px 17px;
      border: 1px solid #535c63; border-radius: 6px;
      background: #0d0f11; color: #fff; text-align: center;
      font-size: 19px; font-weight: 750; font-variant-numeric: tabular-nums;
    }
    .value-input:focus { outline: 2px solid var(--accent); outline-offset: 0; }
    .range {
      position: absolute; bottom: 4px; left: 4px; right: 4px;
      color: #768089; text-align: center; font-size: 9px; pointer-events: none;
    }
    .toggle-control {
      display: flex; align-items: center; justify-content: space-between;
      width: 100%; height: 50px; padding: 0 14px;
      border: 1px solid #515960; border-radius: 6px;
      background: #24292d; color: #d7dce0; cursor: pointer; font-weight: 750;
    }
    .toggle {
      position: relative; width: 52px; height: 28px;
      border-radius: 15px; background: #555e65; transition: background 120ms ease;
    }
    .toggle::after {
      content: ""; position: absolute; top: 4px; left: 4px;
      width: 20px; height: 20px; border-radius: 50%; background: #fff;
      transition: transform 120ms ease;
    }
    .toggle-control[aria-checked="true"] .toggle { background: var(--good); }
    .toggle-control[aria-checked="true"] .toggle::after { transform: translateX(24px); }
    .slider-control {
      display: grid; grid-template-columns: minmax(0, 1fr) 72px;
      align-items: center; gap: 12px; height: 50px;
    }
    .brightness-slider { width: 100%; accent-color: var(--good); cursor: pointer; }
    .slider-value {
      display: flex; align-items: center; justify-content: center;
      height: 42px; border: 1px solid #535c63; border-radius: 6px;
      background: #0d0f11; color: #fff; font-size: 18px;
      font-weight: 750; font-variant-numeric: tabular-nums;
    }
    .card-status { min-height: 18px; margin-top: 7px; color: #7f8991; font-size: 11px; text-align: right; }
    .card-status.ok { color: var(--good); }
    .card-status.fail { color: var(--bad); }
    .empty {
      padding: 28px; border: 1px solid var(--line); border-radius: 7px;
      color: var(--muted); text-align: center;
    }
    details { margin-top: 24px; color: #758088; font-size: 11px; }
    summary { cursor: pointer; }
    .file-path { margin-top: 8px; font-family: ui-monospace, monospace; overflow-wrap: anywhere; }
    button:disabled, input:disabled { cursor: default; opacity: 0.5; }
    @media (max-width: 760px) {
      .app-header { gap: 9px; }
      .connection { margin-left: auto; }
      .icon-button { margin-left: 0; }
      .param-list { grid-template-columns: 1fr; }
      .description { min-height: 0; }
    }
    @media (max-width: 460px) {
      h1 { font-size: 17px; }
      .group-tab { font-size: 11px; }
      .connection span:last-child { max-width: 92px; overflow: hidden; text-overflow: ellipsis; }
      .view-bar { align-items: stretch; flex-direction: column; }
      .view-tabs { width: 100%; }
      .effects { grid-template-columns: 1fr; }
      .number-control { grid-template-columns: 58px minmax(0, 1fr) 58px; }
      .adjust-button, .value-input, .toggle-control { height: 54px; }
      .number-control { height: 54px; }
    }
  </style>
</head>
<body>
  <header class="app-header">
    <div class="brand"><h1>K7 실시간 튜닝</h1></div>
    <div class="connection" id="connection">
      <span id="dot" class="dot"></span><span id="status">연결 확인 중</span>
    </div>
    <button id="reload" class="icon-button" type="button" title="새로고침" aria-label="새로고침">↻</button>
  </header>
  <nav class="group-tabs" aria-label="파라미터 그룹">
    <button class="group-tab active" data-group="steering" type="button">조향</button>
    <button class="group-tab" data-group="driving" type="button">주행 제한</button>
    <button class="group-tab" data-group="adaptive_cruise" type="button">비전 크루즈</button>
    <button class="group-tab" data-group="recording" type="button">주행 기록</button>
    <button class="group-tab" data-group="display" type="button">디스플레이</button>
  </nav>
  <main>
    <div class="view-bar">
      <div class="view-tabs" aria-label="표시 범위">
        <button class="view-tab active" data-view="quick" type="button">빠른 조정</button>
        <button class="view-tab" data-view="all" type="button">전체 설정</button>
      </div>
      <div id="count" class="count"></div>
    </div>
    <div id="group-note" class="group-note"></div>
    <div id="message" role="status"></div>
    <div id="sections"></div>
    <details>
      <summary>파라미터 파일</summary>
      <div id="path" class="file-path"></div>
    </details>
  </main>
  <script>
    let snapshot = null;
    let activeGroup = "steering";
    let activeView = "quick";
    let messageTimer = null;

    const sections = document.getElementById("sections");
    const message = document.getElementById("message");
    const path = document.getElementById("path");
    const dot = document.getElementById("dot");
    const status = document.getElementById("status");
    const connection = document.getElementById("connection");
    const count = document.getElementById("count");
    const groupNote = document.getElementById("group-note");
    const sectionOrder = {
      steering: [
        "기본 토크 제한", "토크 컨트롤러", "조향 반응", "차량 중심 보정",
        "운전자 개입", "속도별 토크 제한", "Smooth steer", "차량 모델",
        "LKAS fault 보호", "고정 조향 한계", "고정 차량 설정", "기타",
      ],
      driving: [
        "빠른 주행 튜닝", "운전자 개입", "상태와 CAN", "데이터 상태",
        "고정 차량 설정", "기타",
      ],
      adaptive_cruise: [
        "동작", "차간 거리", "속도 반응", "복귀 동작", "비전 판정",
        "버튼 송신", "기타",
      ],
      recording: ["기록"],
      display: ["백라이트"],
    };
    const groupNotes = {
      adaptive_cruise: "변경값은 즉시 적용됩니다. 이 기능은 순정 크루즈 버튼만 조절하며 브레이크를 직접 제어하지 않습니다.",
      recording: "기록은 모델 입력과 같은 1280x720 프레임을 사용합니다. 영상·CAN·상태·파라미터가 한 경로에 함께 저장됩니다.",
      display: "전원을 꺼도 영상 파이프라인은 계속 동작합니다. 밝기 값은 다음에 켤 때 그대로 복원됩니다.",
    };

    function setConnection(pids, saved = false, recording = false) {
      const online = pids.length > 0;
      dot.classList.toggle("online", online);
      const process = recording ? "recordd" : "controlsd";
      connection.title = online ? `${process} PID ${pids.join(", ")}` : `${process}가 실행 중이 아닙니다`;
      status.textContent = online
        ? (saved ? (recording ? "기록 설정됨" : "실시간 적용됨") : (recording ? "기록기 연결됨" : "제어 연결됨"))
        : (saved ? "저장됨 · 프로세스 미연결" : "프로세스 미연결");
    }

    function refreshConnection(saved = false) {
      if (activeGroup === "display") {
        const display = snapshot.display_status || {};
        const online = Boolean(display.available) && !display.error;
        dot.classList.toggle("online", online);
        connection.title = display.error || `백라이트 모드: ${display.mode}`;
        status.textContent = online
          ? (saved ? "디스플레이 적용됨" : "백라이트 연결됨")
          : "백라이트 제어 오류";
        return;
      }
      const recording = activeGroup === "recording";
      const pids = recording ? snapshot.recordd_pids : snapshot.controlsd_pids;
      setConnection(pids || [], saved, recording);
    }

    function setMessage(text, failed = false) {
      window.clearTimeout(messageTimer);
      message.textContent = text;
      message.className = text ? `visible${failed ? " failed" : ""}` : "";
      if (text && !failed) {
        messageTimer = window.setTimeout(() => {
          message.textContent = "";
          message.className = "";
        }, 1800);
      }
    }

    function genericMeta(key, value) {
      return {
        label: key,
        section: "기타",
        unit: "",
        step: Number.isInteger(value) ? 1 : 0.01,
        min: -1000000,
        max: 1000000,
        description: "추가 설명이 등록되지 않은 파라미터입니다.",
        increase: "값이 증가합니다.",
        decrease: "값이 감소합니다.",
        quick: false,
        quick_section: "기타",
        quick_order: Number.MAX_SAFE_INTEGER,
      };
    }

    function decimalsForStep(step) {
      const text = String(step);
      if (text.includes("e-")) return Number(text.split("e-")[1]);
      return text.includes(".") ? text.split(".")[1].length : 0;
    }

    function clampAndRound(value, meta) {
      const clamped = Math.min(meta.max, Math.max(meta.min, value));
      const decimals = decimalsForStep(meta.step);
      return Number(clamped.toFixed(decimals));
    }

    async function loadParams(showMessage = false) {
      try {
        const response = await fetch("/api/params", {cache: "no-store"});
        if (!response.ok) throw new Error(await response.text());
        snapshot = await response.json();
        refreshConnection();
        if (showMessage) setMessage("최신 값을 불러왔습니다.");
        render();
      } catch (error) {
        setMessage(`읽기 실패: ${error.message}`, true);
      }
    }

    async function applyValue(key, value, card, input = null) {
      const group = activeGroup;
      card.classList.remove("saved", "error");
      card.classList.add("busy");
      card.querySelectorAll("button, input").forEach(control => control.disabled = true);
      const cardStatus = card.querySelector(".card-status");
      cardStatus.textContent = "적용 중";
      cardStatus.className = "card-status";
      try {
        const response = await fetch(`/api/params/${group}`, {
          method: "PATCH",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({values: {[key]: value}}),
        });
        if (!response.ok) throw new Error(await response.text());
        const update = await response.json();
        snapshot.params[group] = update.params;
        snapshot.controlsd_pids = update.controlsd_pids || update.notified_pids;
        snapshot.recordd_pids = update.recordd_pids || [];
        snapshot.display_status = update.display_status || snapshot.display_status;
        const applied = update.params[key];
        if (input) input.value = String(applied);
        card.classList.add("saved");
        cardStatus.textContent = `적용됨 ${new Date().toLocaleTimeString("ko-KR", {hour12: false})}`;
        cardStatus.className = "card-status ok";
        refreshConnection(true);
        window.setTimeout(() => card.classList.remove("saved"), 900);
        return applied;
      } catch (error) {
        card.classList.add("error");
        cardStatus.textContent = "적용 실패";
        cardStatus.className = "card-status fail";
        setMessage(error.message, true);
        throw error;
      } finally {
        card.classList.remove("busy");
        card.querySelectorAll("button, input").forEach(control => control.disabled = false);
      }
    }

    function createEffects(meta, isBoolean) {
      const effects = document.createElement("div");
      effects.className = "effects";
      const down = document.createElement("div");
      down.className = "effect down";
      const downTitle = document.createElement("b");
      downTitle.textContent = isBoolean ? "끄면" : "값 감소";
      down.append(downTitle, document.createTextNode(meta.decrease));
      const up = document.createElement("div");
      up.className = "effect up";
      const upTitle = document.createElement("b");
      upTitle.textContent = isBoolean ? "켜면" : "값 증가";
      up.append(upTitle, document.createTextNode(meta.increase));
      effects.append(down, up);
      return effects;
    }

    function createNumberControl(key, value, meta, card) {
      const control = document.createElement("div");
      control.className = "number-control";
      const minus = document.createElement("button");
      minus.type = "button";
      minus.className = "adjust-button";
      minus.textContent = "−";
      minus.title = `${meta.step}${meta.unit ? ` ${meta.unit}` : ""} 감소`;
      minus.setAttribute("aria-label", minus.title);
      const valueWrap = document.createElement("div");
      valueWrap.className = "value-wrap";
      const input = document.createElement("input");
      input.className = "value-input";
      input.type = "number";
      input.inputMode = "decimal";
      input.value = String(value);
      input.step = String(meta.step);
      input.min = String(meta.min);
      input.max = String(meta.max);
      input.setAttribute("aria-label", `${meta.label} 현재값`);
      const range = document.createElement("span");
      range.className = "range";
      range.textContent = `${meta.min} ~ ${meta.max}${meta.unit ? ` ${meta.unit}` : ""}`;
      valueWrap.append(input, range);
      const plus = document.createElement("button");
      plus.type = "button";
      plus.className = "adjust-button";
      plus.textContent = "+";
      plus.title = `${meta.step}${meta.unit ? ` ${meta.unit}` : ""} 증가`;
      plus.setAttribute("aria-label", plus.title);

      const adjust = async direction => {
        const current = Number(input.value);
        if (!Number.isFinite(current)) {
          setMessage("숫자를 입력하세요.", true);
          return;
        }
        const next = clampAndRound(current + direction * meta.step, meta);
        input.value = String(next);
        try {
          await applyValue(key, next, card, input);
        } catch (_) {
          input.value = String(snapshot.params[activeGroup][key]);
        }
      };
      minus.addEventListener("click", () => adjust(-1));
      plus.addEventListener("click", () => adjust(1));
      input.addEventListener("keydown", event => {
        if (event.key === "Enter") input.blur();
      });
      input.addEventListener("change", async () => {
        const parsed = Number(input.value);
        if (!Number.isFinite(parsed)) {
          setMessage("숫자를 입력하세요.", true);
          input.value = String(snapshot.params[activeGroup][key]);
          return;
        }
        const next = clampAndRound(parsed, meta);
        input.value = String(next);
        try {
          await applyValue(key, next, card, input);
        } catch (_) {
          input.value = String(snapshot.params[activeGroup][key]);
        }
      });
      control.append(minus, valueWrap, plus);
      return control;
    }

    function createToggleControl(key, value, meta, card) {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "toggle-control";
      button.setAttribute("role", "switch");
      button.setAttribute("aria-checked", String(value));
      const label = document.createElement("span");
      label.textContent = value ? "켜짐" : "꺼짐";
      const toggle = document.createElement("span");
      toggle.className = "toggle";
      button.append(label, toggle);
      button.addEventListener("click", async () => {
        const next = button.getAttribute("aria-checked") !== "true";
        try {
          const applied = await applyValue(key, next, card);
          button.setAttribute("aria-checked", String(applied));
          label.textContent = applied ? "켜짐" : "꺼짐";
        } catch (_) {}
      });
      return button;
    }

    function createSliderControl(key, value, meta, card) {
      const control = document.createElement("div");
      control.className = "slider-control";
      const input = document.createElement("input");
      input.className = "brightness-slider";
      input.type = "range";
      input.min = String(meta.min);
      input.max = String(meta.max);
      input.step = String(meta.step);
      input.value = String(value);
      input.setAttribute("aria-label", `${meta.label} 현재값`);
      const output = document.createElement("output");
      output.className = "slider-value";
      output.textContent = `${value}${meta.unit}`;
      input.addEventListener("input", () => {
        output.textContent = `${input.value}${meta.unit}`;
      });
      input.addEventListener("change", async () => {
        const previous = snapshot.params[activeGroup][key];
        const next = clampAndRound(Number(input.value), meta);
        try {
          const applied = await applyValue(key, next, card, input);
          output.textContent = `${applied}${meta.unit}`;
        } catch (_) {
          input.value = String(previous);
          output.textContent = `${previous}${meta.unit}`;
        }
      });
      control.append(input, output);
      return control;
    }

    function createCard(key, value, meta) {
      const card = document.createElement("article");
      card.className = "param-card";
      const head = document.createElement("div");
      head.className = "param-head";
      const identity = document.createElement("div");
      const title = document.createElement("h3");
      title.className = "param-title";
      title.textContent = meta.label;
      const technicalKey = document.createElement("div");
      technicalKey.className = "param-key";
      technicalKey.textContent = key;
      identity.append(title, technicalKey);
      const unit = document.createElement("div");
      unit.className = "param-unit";
      unit.textContent = meta.unit || (typeof value === "boolean" ? "ON / OFF" : "");
      head.append(identity, unit);
      const description = document.createElement("p");
      description.className = "description";
      description.textContent = meta.description;
      const cardStatus = document.createElement("div");
      cardStatus.className = "card-status";
      card.append(head, description, createEffects(meta, typeof value === "boolean"));
      let editor;
      if (typeof value === "boolean") {
        editor = createToggleControl(key, value, meta, card);
      } else if (meta.control === "slider") {
        editor = createSliderControl(key, value, meta, card);
      } else {
        editor = createNumberControl(key, value, meta, card);
      }
      card.append(editor, cardStatus);
      return card;
    }

    function render() {
      if (!snapshot) return;
      sections.replaceChildren();
      path.textContent = snapshot.paths[activeGroup];
      const note = groupNotes[activeGroup] || "";
      groupNote.textContent = note;
      groupNote.classList.toggle("visible", Boolean(note));
      const params = snapshot.params[activeGroup];
      const metadata = snapshot.metadata[activeGroup] || {};
      const visible = Object.entries(params).filter(([key, value]) => {
        const meta = metadata[key] || genericMeta(key, value);
        return activeView === "all" || meta.quick;
      });
      if (activeView === "quick") {
        visible.sort(([keyA, valueA], [keyB, valueB]) => {
          const metaA = metadata[keyA] || genericMeta(keyA, valueA);
          const metaB = metadata[keyB] || genericMeta(keyB, valueB);
          return (metaA.quick_order ?? Number.MAX_SAFE_INTEGER) -
            (metaB.quick_order ?? Number.MAX_SAFE_INTEGER);
        });
      }
      count.textContent = `${visible.length}개 항목`;
      const grouped = new Map();
      for (const [key, value] of visible) {
        const meta = metadata[key] || genericMeta(key, value);
        const sectionName = activeView === "quick"
          ? (meta.quick_section || meta.section)
          : meta.section;
        if (!grouped.has(sectionName)) grouped.set(sectionName, []);
        grouped.get(sectionName).push([key, value, meta]);
      }
      const orderedGroups = [...grouped.entries()];
      if (activeView === "all") {
        const order = sectionOrder[activeGroup] || [];
        orderedGroups.sort(([nameA], [nameB]) => {
          const rankA = order.includes(nameA) ? order.indexOf(nameA) : order.length;
          const rankB = order.includes(nameB) ? order.indexOf(nameB) : order.length;
          return rankA - rankB;
        });
      }
      for (const [sectionName, entries] of orderedGroups) {
        const section = document.createElement("section");
        section.className = "section";
        const heading = document.createElement("h2");
        heading.className = "section-title";
        heading.append(document.createTextNode(sectionName));
        const sectionCount = document.createElement("span");
        sectionCount.textContent = `${entries.length}개`;
        heading.append(sectionCount);
        const list = document.createElement("div");
        list.className = "param-list";
        for (const [key, value, meta] of entries) {
          list.appendChild(createCard(key, value, meta));
        }
        section.append(heading, list);
        sections.appendChild(section);
      }
      if (!visible.length) {
        const empty = document.createElement("div");
        empty.className = "empty";
        empty.textContent = "빠른 조정 항목이 없습니다.";
        sections.appendChild(empty);
      }
    }

    document.querySelectorAll(".group-tab").forEach(tab => {
      tab.addEventListener("click", () => {
        document.querySelectorAll(".group-tab").forEach(item => item.classList.remove("active"));
        tab.classList.add("active");
        activeGroup = tab.dataset.group;
        refreshConnection();
        render();
        window.scrollTo({top: 0, behavior: "smooth"});
      });
    });
    document.querySelectorAll(".view-tab").forEach(tab => {
      tab.addEventListener("click", () => {
        document.querySelectorAll(".view-tab").forEach(item => item.classList.remove("active"));
        tab.classList.add("active");
        activeView = tab.dataset.view;
        render();
      });
    });
    document.getElementById("reload").addEventListener("click", () => loadParams(true));
    loadParams();
  </script>
</body>
</html>
"""


def create_app(store: ParamStore | None = None) -> FastAPI:
    param_store = store or ParamStore(display_controller=DisplayBacklight())
    application = FastAPI(title="K7 parameter server", docs_url="/docs")

    @application.get("/", response_class=HTMLResponse)
    def index() -> str:
        return HTML

    @application.get("/api/params")
    def get_params() -> Dict[str, Any]:
        try:
            return param_store.snapshot()
        except (KeyError, ValueError) as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    @application.patch("/api/params/{group}")
    def patch_params(group: str, patch: ParamPatch) -> Dict[str, Any]:
        try:
            return param_store.update(group, patch.values)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=f"unknown parameter: {exc}") from exc
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except RuntimeError as exc:
            raise HTTPException(status_code=503, detail=str(exc)) from exc

    return application


app = create_app()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=os.environ.get("K230_PARAM_HOST", "0.0.0.0"))
    parser.add_argument(
        "--port", type=int, default=int(os.environ.get("K230_PARAM_PORT", "8080"))
    )
    args = parser.parse_args()
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
