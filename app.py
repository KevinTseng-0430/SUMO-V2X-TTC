#!/usr/bin/env python3
"""V2X rear-end warning service for controlled MEC-versus-WAN trials.

Deploy this exact file in both locations. Select the architecture with
DEPLOYMENT_MODE=mec or DEPLOYMENT_MODE=wan. Both profiles use the same risk
algorithm; only the explicitly reported response delay differs.
"""

from __future__ import annotations

import asyncio
import math
import os
import time
from dataclasses import dataclass
from typing import Any

from fastapi import FastAPI, Request, Response
from pydantic import BaseModel


SERVICE_VERSION = "2.0.0"
LEADER_ID = "leader_car"
FOLLOWER_ID = "follower_car"


def env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        value = float(raw)
    except ValueError as exc:
        raise RuntimeError(f"{name} must be a number, got {raw!r}") from exc
    if not math.isfinite(value) or value < 0:
        raise RuntimeError(f"{name} must be a finite non-negative number")
    return value


@dataclass(frozen=True)
class ServiceConfig:
    architecture: str
    ttc_warning_threshold_s: float
    ttc_caution_threshold_s: float
    added_delay_ms: float


def load_config() -> ServiceConfig:
    architecture = os.environ.get("DEPLOYMENT_MODE", "mec").strip().lower()
    if architecture not in {"mec", "wan"}:
        raise RuntimeError("DEPLOYMENT_MODE must be either 'mec' or 'wan'")

    default_delay_ms = 0.0 if architecture == "mec" else 500.0
    profile_delay_name = (
        "MEC_ADDED_DELAY_MS" if architecture == "mec" else "WAN_ADDED_DELAY_MS"
    )
    profile_delay_ms = env_float(profile_delay_name, default_delay_ms)
    added_delay_ms = env_float("ADDED_DELAY_MS", profile_delay_ms)
    warning_threshold = env_float("TTC_WARNING_THRESHOLD_S", 4.0)
    caution_threshold = env_float("TTC_CAUTION_THRESHOLD_S", 5.0)
    if warning_threshold <= 0:
        raise RuntimeError("TTC_WARNING_THRESHOLD_S must be greater than zero")
    if caution_threshold < warning_threshold:
        raise RuntimeError(
            "TTC_CAUTION_THRESHOLD_S must be greater than or equal to "
            "TTC_WARNING_THRESHOLD_S"
        )

    return ServiceConfig(
        architecture=architecture,
        ttc_warning_threshold_s=warning_threshold,
        ttc_caution_threshold_s=caution_threshold,
        added_delay_ms=added_delay_ms,
    )


CONFIG = load_config()
app = FastAPI(
    title="SUMO V2X Collision Warning Service",
    version=SERVICE_VERSION,
)


class VehicleState(BaseModel):
    vehicle_id: str
    x: float
    y: float
    speed: float
    accel: float | None = 0.0
    lane_id: str
    lane_pos: float
    length: float = 5.0


class V2XReport(BaseModel):
    scenario: str
    ue_id: str
    sim_time: float
    client_send_ts: float
    vehicles: list[VehicleState]
    run_id: str | None = None
    sequence: int | None = None
    event_state: str | None = None


def make_json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: make_json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [make_json_safe(item) for item in value]
    return value


def invalid_result(reason: str) -> dict[str, Any]:
    return {
        "valid": False,
        "reason": reason,
        "warning": False,
        "risk_level": "unknown",
        "ttc": None,
        "gap": None,
        "relative_speed": None,
        "leader_id": "",
        "follower_id": "",
        "same_lane": False,
    }


def select_vehicle_pair(
    vehicles: list[VehicleState],
) -> tuple[VehicleState, VehicleState] | None:
    vehicle_map = {vehicle.vehicle_id: vehicle for vehicle in vehicles}
    if LEADER_ID in vehicle_map and FOLLOWER_ID in vehicle_map:
        return vehicle_map[LEADER_ID], vehicle_map[FOLLOWER_ID]

    lane_groups: dict[str, list[VehicleState]] = {}
    for vehicle in vehicles:
        lane_groups.setdefault(vehicle.lane_id, []).append(vehicle)
    candidate_groups = [
        group for group in lane_groups.values() if len(group) >= 2
    ]
    if not candidate_groups:
        return None
    candidate_group = max(candidate_groups, key=len)
    ordered = sorted(
        candidate_group,
        key=lambda vehicle: vehicle.lane_pos,
        reverse=True,
    )
    return ordered[0], ordered[1]


def compute_rear_end_ttc(vehicles: list[VehicleState]) -> dict[str, Any]:
    if len(vehicles) < 2:
        return invalid_result("need at least two vehicles")

    pair = select_vehicle_pair(vehicles)
    if pair is None:
        return invalid_result("no two vehicles on the same lane")
    leader, follower = pair

    same_lane = leader.lane_id == follower.lane_id
    if not same_lane:
        result = invalid_result("leader and follower are not on the same lane")
        result.update(
            {
                "risk_level": "safe",
                "leader_id": leader.vehicle_id,
                "follower_id": follower.vehicle_id,
            }
        )
        return result

    # SUMO lane_pos is the front-bumper position. The leader's rear bumper is
    # therefore leader.lane_pos - leader.length.
    gap = leader.lane_pos - leader.length - follower.lane_pos
    relative_speed = follower.speed - leader.speed

    if gap <= 0:
        ttc: float | None = 0.0
        warning = True
        risk_level = "collision"
    elif relative_speed > 0.1:
        ttc = gap / relative_speed
        warning = ttc <= CONFIG.ttc_warning_threshold_s
        if ttc <= 1.0:
            risk_level = "critical"
        elif warning:
            risk_level = "warning"
        elif ttc <= CONFIG.ttc_caution_threshold_s:
            risk_level = "caution"
        else:
            risk_level = "safe"
    else:
        ttc = None
        warning = False
        risk_level = "safe"

    return make_json_safe(
        {
            "valid": True,
            "leader_id": leader.vehicle_id,
            "follower_id": follower.vehicle_id,
            "same_lane": True,
            "gap": gap,
            "relative_speed": relative_speed,
            "ttc": ttc,
            "risk_level": risk_level,
            "warning": warning,
            "ttc_warning_threshold_s": CONFIG.ttc_warning_threshold_s,
        }
    )


def client_ip(request: Request) -> str:
    forwarded = request.headers.get("x-forwarded-for", "")
    if forwarded:
        return forwarded.split(",", 1)[0].strip()
    return request.client.host if request.client else ""


@app.get("/health")
@app.get("/status")
def status() -> dict[str, Any]:
    return {
        "service": "v2x-collision-warning",
        "version": SERVICE_VERSION,
        "status": "ok",
        "architecture": CONFIG.architecture,
        "configured_delay_ms": CONFIG.added_delay_ms,
        "ttc_warning_threshold_s": CONFIG.ttc_warning_threshold_s,
        "ttc_caution_threshold_s": CONFIG.ttc_caution_threshold_s,
        "server_ts": time.time(),
    }


@app.post("/report")
@app.post("/mec/v2x/sumo/report")
async def report(
    payload: V2XReport,
    request: Request,
    response: Response,
) -> dict[str, Any]:
    server_recv_ts = time.time()
    total_started = time.perf_counter()

    compute_started = time.perf_counter()
    result = compute_rear_end_ttc(payload.vehicles)
    compute_delay_ms = (time.perf_counter() - compute_started) * 1000.0

    # asyncio.sleep keeps the service concurrent while creating a controlled,
    # explicitly reported WAN delay. It is zero in the default MEC profile.
    if CONFIG.added_delay_ms > 0:
        await asyncio.sleep(CONFIG.added_delay_ms / 1000.0)

    server_send_ts = time.time()
    proc_delay_ms = (time.perf_counter() - total_started) * 1000.0
    source_ip = client_ip(request)

    response.headers["X-V2X-Architecture"] = CONFIG.architecture
    response.headers["X-V2X-Configured-Delay-Ms"] = str(CONFIG.added_delay_ms)

    ttc = result.get("ttc")
    gap = result.get("gap")
    relative_speed = result.get("relative_speed")
    ttc_text = f"{ttc:.3f}s" if isinstance(ttc, (int, float)) else "null"
    gap_text = f"{gap:.2f}" if isinstance(gap, (int, float)) else "null"
    relative_text = (
        f"{relative_speed:.2f}"
        if isinstance(relative_speed, (int, float))
        else "null"
    )
    print(
        f"[V2X-{CONFIG.architecture.upper()}] "
        f"client={source_ip} | "
        f"run={payload.run_id or '-'} | "
        f"seq={payload.sequence if payload.sequence is not None else '-'} | "
        f"sim_time={payload.sim_time:.1f}s | "
        f"risk={result.get('risk_level', 'unknown')} | "
        f"warning={result.get('warning', False)} | "
        f"ttc={ttc_text} | gap={gap_text}m | "
        f"v_rel={relative_text}m/s | "
        f"compute={compute_delay_ms:.3f}ms | "
        f"added={CONFIG.added_delay_ms:.1f}ms | "
        f"total={proc_delay_ms:.3f}ms",
        flush=True,
    )

    return make_json_safe(
        {
            "service": "v2x-collision-warning",
            "version": SERVICE_VERSION,
            "architecture": CONFIG.architecture,
            "configured_delay_ms": CONFIG.added_delay_ms,
            "compute_delay_ms": compute_delay_ms,
            "scenario": payload.scenario,
            "ue_id": payload.ue_id,
            "run_id": payload.run_id,
            "sequence": payload.sequence,
            "event_state": payload.event_state,
            "sim_time": payload.sim_time,
            "client_send_ts": payload.client_send_ts,
            "server_recv_ts": server_recv_ts,
            "server_send_ts": server_send_ts,
            "proc_delay_ms": proc_delay_ms,
            "client_ip": source_ip,
            "result": result,
        }
    )
