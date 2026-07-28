from fastapi import FastAPI, Request
from pydantic import BaseModel
from typing import List, Optional
import time
import math

app = FastAPI()

TTC_WARNING_THRESHOLD = 2.0
TTC_CAUTION_THRESHOLD = 5.0

def json_safe_float(value):
    if isinstance(value, float):
        if math.isfinite(value):
            return value
        return None
    return value


class VehicleState(BaseModel):
    vehicle_id: str
    x: float
    y: float
    speed: float
    accel: Optional[float] = 0.0
    lane_id: str
    lane_pos: float
    length: float = 5.0


class V2XReport(BaseModel):
    scenario: str
    ue_id: str
    sim_time: float
    client_send_ts: float
    vehicles: List[VehicleState]


@app.get("/status")
def status():
    return {
        "service": "mec-v2x",
        "status": "ok",
        "ts": time.time()
    }


def compute_rear_end_ttc(vehicles: List[VehicleState]):
    if len(vehicles) < 2:
        return {
            "valid": False,
            "reason": "need at least two vehicles",
            "warning": False,
            "risk_level": "unknown",
            "ttc": None,
            "gap": None,
            "relative_speed": None,
        }

    vmap = {v.vehicle_id: v for v in vehicles}

    if "leader_car" in vmap and "follower_car" in vmap:
        leader = vmap["leader_car"]
        follower = vmap["follower_car"]
    else:
        same_lane_groups = {}
        for v in vehicles:
            same_lane_groups.setdefault(v.lane_id, []).append(v)

        candidate_group = max(same_lane_groups.values(), key=len)

        if len(candidate_group) < 2:
            return {
                "valid": False,
                "reason": "no two vehicles on same lane",
                "warning": False,
                "risk_level": "unknown",
                "ttc": None,
                "gap": None,
                "relative_speed": None,
            }

        sorted_cars = sorted(
            candidate_group,
            key=lambda v: v.lane_pos,
            reverse=True
        )
        leader = sorted_cars[0]
        follower = sorted_cars[1]

    same_lane = leader.lane_id == follower.lane_id

    if not same_lane:
        return {
            "valid": False,
            "reason": "leader and follower are not on the same lane",
            "warning": False,
            "risk_level": "safe",
            "ttc": None,
            "gap": None,
            "relative_speed": None,
            "leader_id": leader.vehicle_id,
            "follower_id": follower.vehicle_id,
            "same_lane": False,
        }

    gap = leader.lane_pos - follower.lane_pos - leader.length
    relative_speed = follower.speed - leader.speed

    if gap <= 0:
        ttc = 0.0
        warning = True
        risk_level = "collision"
    elif relative_speed > 0.1:
        ttc = gap / relative_speed

        if ttc < 1.0:
            risk_level = "critical"
        elif ttc < TTC_WARNING_THRESHOLD:
            risk_level = "warning"
        elif ttc < TTC_CAUTION_THRESHOLD:
            risk_level = "caution"
        else:
            risk_level = "safe"

        warning = ttc < TTC_WARNING_THRESHOLD
    else:
        ttc = None
        warning = False
        risk_level = "safe"

    return {
        "valid": True,
        "leader_id": leader.vehicle_id,
        "follower_id": follower.vehicle_id,
        "same_lane": same_lane,
        "gap": json_safe_float(gap),
        "relative_speed": json_safe_float(relative_speed),
        "ttc": json_safe_float(ttc),
        "risk_level": risk_level,
        "warning": warning,
    }
    
def make_json_safe(obj):
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None

    if isinstance(obj, dict):
        return {key: make_json_safe(value) for key, value in obj.items()}

    if isinstance(obj, list):
        return [make_json_safe(item) for item in obj]

    return obj


@app.post("/report")
async def report(payload: V2XReport, request: Request):
    recv_ts = time.time()
    proc_start = time.perf_counter()

    result = compute_rear_end_ttc(payload.vehicles)

    proc_delay_ms = (time.perf_counter() - proc_start) * 1000.0
    server_send_ts = time.time()

    risk_level = result.get("risk_level", "unknown")
    warning = result.get("warning", False)
    ttc = result.get("ttc", None)
    gap = result.get("gap", None)
    relative_speed = result.get("relative_speed", None)

    ttc_str = f"{ttc:.3f}" if isinstance(ttc, (int, float)) else "null"
    gap_str = f"{gap:.2f}" if isinstance(gap, (int, float)) else "null"
    rel_speed_str = f"{relative_speed:.2f}" if isinstance(relative_speed, (int, float)) else "null"

    print(
        f"[MEC-V2X] "
        f"client={request.client.host} | "
        f"sim_time={payload.sim_time:.1f}s | "
        f"ue={payload.ue_id} | "
        f"risk={risk_level} | "
        f"warning={warning} | "
        f"ttc={ttc_str}s | "
        f"gap={gap_str}m | "
        f"v_rel={rel_speed_str}m/s | "
        f"proc={proc_delay_ms:.3f}ms",
        flush=True
    )

    response = {
        "scenario": payload.scenario,
        "ue_id": payload.ue_id,
        "sim_time": payload.sim_time,
        "client_send_ts": payload.client_send_ts,
        "server_recv_ts": recv_ts,
        "server_send_ts": server_send_ts,
        "proc_delay_ms": proc_delay_ms,
        "client_ip": request.client.host,
        "result": result,
    }

    return make_json_safe(response)
