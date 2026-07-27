#!/usr/bin/env python3
"""Closed-loop SUMO rear-end warning scenario with MEC reporting.

The simulation clock is paced against wall time by default. HTTP requests run
in background workers so MEC/network latency affects the simulation time at
which a warning becomes available instead of blocking TraCI.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import subprocess
import sys
import time
import uuid
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.parse import urlparse


BASE_DIR = Path(__file__).resolve().parent
DEFAULT_SUMO_CFG = BASE_DIR / "rear_end.sumocfg"
DEFAULT_OUTPUT_DIR = BASE_DIR / "outputs"
DEFAULT_MEC_URL = "http://172.16.6.100/mec/v2x/sumo/report"
DEFAULT_UE_INTERFACE = "uesimtun0"

LEADER_ID = "leader_car"
FOLLOWER_ID = "follower_car"
DEFAULT_SCENARIO = "rear_end_emergency_brake"
DEFAULT_UE_ID = "ueransim-ue-001"


def load_traci():
    """Import TraCI from Python or from the tools bundled with system SUMO."""
    try:
        import traci  # type: ignore

        return traci
    except ModuleNotFoundError:
        candidates: list[Path] = []
        sumo_home = os.environ.get("SUMO_HOME")
        if sumo_home:
            candidates.append(Path(sumo_home) / "tools")
        candidates.extend(
            [
                Path("/usr/share/sumo/tools"),
                Path("/usr/local/share/sumo/tools"),
            ]
        )

        for tools_dir in candidates:
            if (tools_dir / "traci").is_dir():
                sys.path.insert(0, str(tools_dir))
                import traci  # type: ignore

                return traci

    raise RuntimeError(
        "Cannot import TraCI. Install the SUMO tools package or set "
        "SUMO_HOME to the SUMO installation directory."
    )


traci = load_traci()


@dataclass
class PostResult:
    sequence: int
    sim_time: float
    client_send_ts: float
    client_recv_ts: float
    rtt_ms: float
    ok: bool
    http_status: int | None = None
    response: dict[str, Any] | None = None
    error: str = ""
    raw: str = ""


def post_json_via_curl(
    *,
    sequence: int,
    sim_time: float,
    client_send_ts: float,
    payload: dict[str, Any],
    url: str,
    interface: str,
    timeout: float,
) -> PostResult:
    """POST JSON while binding curl to the UE tunnel when one is configured."""
    command = [
        "curl",
        "--silent",
        "--show-error",
        "--connect-timeout",
        str(min(timeout, 1.0)),
        "--max-time",
        str(timeout),
        "--header",
        "Content-Type: application/json",
        "--request",
        "POST",
        "--data-binary",
        "@-",
        "--write-out",
        "\n%{http_code}",
    ]
    if interface:
        command.extend(["--interface", interface])
    command.append(url)

    started = time.perf_counter()
    completed = subprocess.run(
        command,
        input=json.dumps(payload, separators=(",", ":")),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    client_recv_ts = time.time()
    rtt_ms = (time.perf_counter() - started) * 1000.0

    body = completed.stdout
    http_status: int | None = None
    if "\n" in body:
        body, status_text = body.rsplit("\n", 1)
        try:
            http_status = int(status_text)
        except ValueError:
            pass

    if completed.returncode != 0:
        return PostResult(
            sequence=sequence,
            sim_time=sim_time,
            client_send_ts=client_send_ts,
            client_recv_ts=client_recv_ts,
            rtt_ms=rtt_ms,
            ok=False,
            http_status=http_status,
            error=completed.stderr.strip() or f"curl exited {completed.returncode}",
            raw=body[:500],
        )

    if http_status is None or not 200 <= http_status < 300:
        return PostResult(
            sequence=sequence,
            sim_time=sim_time,
            client_send_ts=client_send_ts,
            client_recv_ts=client_recv_ts,
            rtt_ms=rtt_ms,
            ok=False,
            http_status=http_status,
            error=f"HTTP status {http_status}",
            raw=body[:500],
        )

    try:
        response = json.loads(body)
    except json.JSONDecodeError as exc:
        return PostResult(
            sequence=sequence,
            sim_time=sim_time,
            client_send_ts=client_send_ts,
            client_recv_ts=client_recv_ts,
            rtt_ms=rtt_ms,
            ok=False,
            http_status=http_status,
            error=f"invalid JSON response: {exc}",
            raw=body[:500],
        )

    if not isinstance(response, dict):
        return PostResult(
            sequence=sequence,
            sim_time=sim_time,
            client_send_ts=client_send_ts,
            client_recv_ts=client_recv_ts,
            rtt_ms=rtt_ms,
            ok=False,
            http_status=http_status,
            error="JSON response must be an object",
            raw=body[:500],
        )

    return PostResult(
        sequence=sequence,
        sim_time=sim_time,
        client_send_ts=client_send_ts,
        client_recv_ts=client_recv_ts,
        rtt_ms=rtt_ms,
        ok=True,
        http_status=http_status,
        response=response,
        raw=body[:500],
    )


class MecClient:
    """Small bounded pool that drops new frames instead of building stale work."""

    def __init__(
        self,
        *,
        url: str,
        interface: str,
        timeout: float,
        max_inflight: int,
    ) -> None:
        self.url = url
        self.interface = interface
        self.timeout = timeout
        self.max_inflight = max_inflight
        self.dropped_frames = 0
        self._executor = ThreadPoolExecutor(
            max_workers=max_inflight,
            thread_name_prefix="mec-post",
        )
        self._pending: dict[Future[PostResult], int] = {}

    def submit(
        self,
        *,
        sequence: int,
        sim_time: float,
        client_send_ts: float,
        payload: dict[str, Any],
    ) -> bool:
        if len(self._pending) >= self.max_inflight:
            self.dropped_frames += 1
            return False

        future = self._executor.submit(
            post_json_via_curl,
            sequence=sequence,
            sim_time=sim_time,
            client_send_ts=client_send_ts,
            payload=payload,
            url=self.url,
            interface=self.interface,
            timeout=self.timeout,
        )
        self._pending[future] = sequence
        return True

    def collect(self) -> list[PostResult]:
        completed: list[PostResult] = []
        for future in list(self._pending):
            if not future.done():
                continue
            self._pending.pop(future)
            try:
                completed.append(future.result())
            except Exception as exc:  # pragma: no cover - worker safeguard
                completed.append(
                    PostResult(
                        sequence=-1,
                        sim_time=math.nan,
                        client_send_ts=math.nan,
                        client_recv_ts=time.time(),
                        rtt_ms=math.nan,
                        ok=False,
                        error=f"MEC worker failed: {exc}",
                    )
                )
        return sorted(completed, key=lambda item: item.sequence)

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    def close(self) -> list[PostResult]:
        self._executor.shutdown(wait=True, cancel_futures=False)
        return self.collect()


def safe_acceleration(vehicle_id: str) -> float:
    try:
        return float(traci.vehicle.getAcceleration(vehicle_id))
    except Exception:
        return 0.0


def vehicle_state(vehicle_id: str) -> dict[str, Any]:
    x, y = traci.vehicle.getPosition(vehicle_id)
    return {
        "vehicle_id": vehicle_id,
        "x": float(x),
        "y": float(y),
        "speed": float(traci.vehicle.getSpeed(vehicle_id)),
        "accel": safe_acceleration(vehicle_id),
        "lane_id": traci.vehicle.getLaneID(vehicle_id),
        "lane_pos": float(traci.vehicle.getLanePosition(vehicle_id)),
        "length": float(traci.vehicle.getLength(vehicle_id)),
    }


def local_ground_truth(
    leader: dict[str, Any],
    follower: dict[str, Any],
    *,
    leader_decel: float,
    follower_decel: float,
) -> dict[str, float]:
    same_lane = leader["lane_id"] == follower["lane_id"]
    if not same_lane:
        return {
            "gap": math.nan,
            "relative_speed": math.nan,
            "ttc": math.inf,
            "stopping_margin": math.nan,
        }

    gap = (
        float(leader["lane_pos"])
        - float(leader["length"])
        - float(follower["lane_pos"])
    )
    relative_speed = float(follower["speed"]) - float(leader["speed"])
    ttc = gap / relative_speed if gap >= 0 and relative_speed > 1e-6 else math.inf
    leader_stop_distance = float(leader["speed"]) ** 2 / (2.0 * leader_decel)
    follower_stop_distance = float(follower["speed"]) ** 2 / (2.0 * follower_decel)
    stopping_margin = gap + leader_stop_distance - follower_stop_distance
    return {
        "gap": gap,
        "relative_speed": relative_speed,
        "ttc": ttc,
        "stopping_margin": stopping_margin,
    }


def finite_or_none(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def finite_or_empty(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return ""
    return value


def interface_exists(interface: str) -> bool:
    if not interface:
        return True
    if not re.fullmatch(r"[A-Za-z0-9_.:-]+", interface):
        return False
    return (Path("/sys/class/net") / interface).exists()


def validate_runtime(args: argparse.Namespace) -> None:
    if not args.sumo_cfg.is_file():
        raise RuntimeError(f"SUMO configuration not found: {args.sumo_cfg}")
    if args.mode == "mec":
        parsed = urlparse(args.mec_url)
        if parsed.scheme not in {"http", "https"} or not parsed.netloc:
            raise RuntimeError(f"Invalid MEC URL: {args.mec_url}")
        if args.ue_interface and not interface_exists(args.ue_interface):
            raise RuntimeError(
                f"Network interface '{args.ue_interface}' does not exist. "
                "Start the UE tunnel first, choose another --ue-interface, "
                "or pass --ue-interface '' to use the normal routing table."
            )
        if not shutil_which("curl"):
            raise RuntimeError("curl is required for MEC reporting")


def shutil_which(command: str) -> str | None:
    """Small local replacement to keep startup dependencies explicit."""
    for directory in os.environ.get("PATH", "").split(os.pathsep):
        candidate = Path(directory) / command
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return None


def event_state(
    *,
    leader_braking: bool,
    warning_received: bool,
    follower_braking: bool,
    collision: bool,
) -> str:
    if collision:
        return "collision"
    if follower_braking:
        return "follower_braking"
    if warning_received:
        return "warning_received"
    if leader_braking:
        return "leader_emergency_braking"
    return "cruise"


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: finite_or_empty(value) for key, value in row.items()})


def collision_ids() -> list[str]:
    try:
        return list(traci.simulation.getCollidingVehiclesIDList())
    except Exception:
        return []


def setup_gui() -> None:
    try:
        view_id = traci.gui.DEFAULT_VIEW
        traci.gui.setSchema(view_id, "v2x_rear_end")
        traci.gui.trackVehicle(view_id, FOLLOWER_ID)
        traci.gui.setZoom(view_id, 650.0)
    except Exception as exc:
        print(f"[warn] GUI setup skipped: {exc}")


def set_warning_visual(active: bool) -> None:
    try:
        if active:
            traci.vehicle.setColor(FOLLOWER_ID, (255, 190, 35, 255))
            traci.poi.setColor("emergency_brake_zone", (255, 55, 40, 255))
        else:
            traci.vehicle.setColor(FOLLOWER_ID, (30, 110, 235, 255))
    except Exception:
        pass


def network_row(result: PostResult, current_sim_time: float) -> dict[str, Any]:
    response = result.response or {}
    mec_result = response.get("result", {})
    if not isinstance(mec_result, dict):
        mec_result = {}
    return {
        "sequence": result.sequence,
        "report_sim_time": result.sim_time,
        "processed_sim_time": current_sim_time,
        "client_send_ts": result.client_send_ts,
        "client_recv_ts": result.client_recv_ts,
        "ok": result.ok,
        "http_status": result.http_status,
        "rtt_ms": result.rtt_ms,
        "error": result.error,
        "server_recv_ts": response.get("server_recv_ts", ""),
        "server_send_ts": response.get("server_send_ts", ""),
        "proc_delay_ms": response.get("proc_delay_ms", ""),
        "client_ip_seen_by_mec": response.get("client_ip", ""),
        "gap": finite_or_empty(mec_result.get("gap", "")),
        "relative_speed": finite_or_empty(mec_result.get("relative_speed", "")),
        "ttc": finite_or_empty(mec_result.get("ttc", "")),
        "risk_level": mec_result.get("risk_level", ""),
        "warning": mec_result.get("warning", ""),
        "leader_id": mec_result.get("leader_id", ""),
        "follower_id": mec_result.get("follower_id", ""),
        "accepted_warning": False,
        "stale": False,
        "raw": result.raw,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a SUMO rear-end warning scenario through a MEC endpoint."
    )
    parser.add_argument(
        "--mode",
        choices=("mec", "baseline", "mec-timeout"),
        default="mec",
        help="mec uses live responses; baseline models driver reaction only; "
        "mec-timeout exercises the no-response fallback.",
    )
    parser.add_argument(
        "--gui",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Open SUMO GUI (default: true).",
    )
    parser.add_argument(
        "--realtime",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Pace simulation time against the wall clock (default: true).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run a fast headless baseline without contacting MEC.",
    )
    parser.add_argument("--sumo-cfg", type=Path, default=DEFAULT_SUMO_CFG)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--run-id", default="")
    parser.add_argument(
        "--mec-url",
        default=os.environ.get("MEC_URL", DEFAULT_MEC_URL),
    )
    parser.add_argument(
        "--ue-interface",
        default=os.environ.get("UE_INTERFACE", DEFAULT_UE_INTERFACE),
    )
    parser.add_argument("--ue-id", default=os.environ.get("UE_ID", DEFAULT_UE_ID))
    parser.add_argument("--scenario", default=DEFAULT_SCENARIO)
    parser.add_argument("--step-length", type=float, default=0.1)
    parser.add_argument("--end-time", type=float, default=15.0)
    parser.add_argument("--send-rate", type=float, default=10.0)
    parser.add_argument("--mec-timeout", type=float, default=2.0)
    parser.add_argument("--max-inflight", type=int, default=4)
    parser.add_argument("--max-response-age", type=float, default=1.0)
    parser.add_argument("--brake-time", type=float, default=5.0)
    parser.add_argument("--cruise-speed", type=float, default=20.0)
    parser.add_argument("--leader-decel", type=float, default=8.0)
    parser.add_argument("--follower-decel", type=float, default=8.0)
    parser.add_argument("--warning-reaction-time", type=float, default=0.3)
    parser.add_argument("--fallback-reaction-time", type=float, default=1.5)
    args = parser.parse_args()

    if args.dry_run:
        args.mode = "baseline"
        args.gui = False
        args.realtime = False

    if not args.sumo_cfg.is_absolute():
        args.sumo_cfg = (BASE_DIR / args.sumo_cfg).resolve()
    if not args.output_dir.is_absolute():
        args.output_dir = (BASE_DIR / args.output_dir).resolve()

    positive_values = {
        "--step-length": args.step_length,
        "--end-time": args.end_time,
        "--send-rate": args.send_rate,
        "--mec-timeout": args.mec_timeout,
        "--max-response-age": args.max_response_age,
        "--leader-decel": args.leader_decel,
        "--follower-decel": args.follower_decel,
    }
    invalid = [name for name, value in positive_values.items() if value <= 0]
    if invalid:
        parser.error(f"values must be positive: {', '.join(invalid)}")
    if args.max_inflight < 1:
        parser.error("--max-inflight must be at least 1")
    if args.brake_time >= args.end_time:
        parser.error("--brake-time must be earlier than --end-time")
    return args


def run(args: argparse.Namespace) -> dict[str, Any]:
    validate_runtime(args)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    run_id = args.run_id or (
        time.strftime("%Y%m%dT%H%M%S") + "-" + uuid.uuid4().hex[:6]
    )
    prefix = args.output_dir / run_id
    step_csv = Path(f"{prefix}_steps.csv")
    network_csv = Path(f"{prefix}_network.csv")
    summary_json = Path(f"{prefix}_summary.json")

    sumo_binary = "sumo-gui" if args.gui else "sumo"
    command = [
        sumo_binary,
        "-c",
        str(args.sumo_cfg),
        "--step-length",
        str(args.step_length),
        "--end",
        str(args.end_time),
        "--fcd-output",
        f"{prefix}_fcd.xml",
        "--tripinfo-output",
        f"{prefix}_tripinfo.xml",
        "--collision-output",
        f"{prefix}_collisions.xml",
        "--start",
        "--quit-on-end",
    ]
    if args.gui:
        command.extend(["--delay", "0"])

    mec_client: MecClient | None = None
    if args.mode == "mec":
        mec_client = MecClient(
            url=args.mec_url,
            interface=args.ue_interface,
            timeout=args.mec_timeout,
            max_inflight=args.max_inflight,
        )

    print(
        f"[run] id={run_id} mode={args.mode} gui={args.gui} "
        f"realtime={args.realtime}"
    )
    if mec_client:
        interface_label = args.ue_interface or "<routing-table>"
        print(f"[run] MEC={args.mec_url} interface={interface_label}")

    rows: list[dict[str, Any]] = []
    network_rows: list[dict[str, Any]] = []
    sequence = 0
    next_send_time = 0.0
    latest_network: dict[str, Any] = {}

    leader_brake_triggered = False
    leader_brake_sim_time: float | None = None
    leader_stop_time: float | None = None
    warning_received = False
    warning_sequence: int | None = None
    warning_report_sim_time: float | None = None
    warning_received_sim_time: float | None = None
    warning_received_wall_ts: float | None = None
    follower_brake_scheduled: float | None = None
    follower_brake_triggered = False
    follower_brake_sim_time: float | None = None
    follower_stop_time: float | None = None
    fallback_used = False
    seen_collisions: set[str] = set()
    dropped_send_frames = 0
    started = False
    wall_origin = 0.0
    trailing_results: list[PostResult] = []

    try:
        traci.start(command)
        started = True
        wall_origin = time.perf_counter()

        while (
            traci.simulation.getMinExpectedNumber() > 0
            and traci.simulation.getTime() < args.end_time
        ):
            traci.simulationStep()
            sim_time = float(traci.simulation.getTime())

            if args.realtime:
                target_wall = wall_origin + sim_time
                remaining = target_wall - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)

            vehicle_ids = set(traci.vehicle.getIDList())
            if LEADER_ID not in vehicle_ids or FOLLOWER_ID not in vehicle_ids:
                continue

            if not rows:
                traci.vehicle.setSpeedMode(FOLLOWER_ID, 0)
                traci.vehicle.setColor(LEADER_ID, (220, 45, 45, 255))
                set_warning_visual(False)
                if args.gui:
                    setup_gui()
                print(
                    f"[event] t={sim_time:.1f}s vehicles ready; "
                    f"follower safety override enabled for controlled trial"
                )

            if mec_client:
                for result in mec_client.collect():
                    nrow = network_row(result, sim_time)
                    response_age = sim_time - result.sim_time
                    nrow["stale"] = (
                        not math.isfinite(response_age)
                        or response_age > args.max_response_age
                    )
                    network_rows.append(nrow)
                    latest_network = nrow

                    response = result.response or {}
                    mec_result = response.get("result", {})
                    if not isinstance(mec_result, dict):
                        mec_result = {}
                    leader_match = mec_result.get("leader_id") in {
                        None,
                        "",
                        LEADER_ID,
                    }
                    follower_match = mec_result.get("follower_id") in {
                        None,
                        "",
                        FOLLOWER_ID,
                    }
                    warning_valid = (
                        result.ok
                        and mec_result.get("warning") is True
                        and not nrow["stale"]
                        and leader_match
                        and follower_match
                    )

                    if warning_valid and not warning_received:
                        warning_received = True
                        warning_sequence = result.sequence
                        warning_report_sim_time = result.sim_time
                        warning_received_sim_time = sim_time
                        warning_received_wall_ts = result.client_recv_ts
                        proposed_brake = sim_time + args.warning_reaction_time
                        if (
                            not follower_brake_triggered
                            and (
                                follower_brake_scheduled is None
                                or proposed_brake < follower_brake_scheduled
                            )
                        ):
                            follower_brake_scheduled = proposed_brake
                            fallback_used = False
                            nrow["accepted_warning"] = True
                        set_warning_visual(True)
                        if nrow["accepted_warning"]:
                            print(
                                f"[event] t={sim_time:.1f}s MEC warning accepted "
                                f"(seq={result.sequence}, RTT={result.rtt_ms:.1f} ms); "
                                f"driver brake scheduled at "
                                f"t={follower_brake_scheduled:.1f}s"
                            )
                        else:
                            print(
                                f"[event] t={sim_time:.1f}s MEC warning arrived "
                                f"too late to replace fallback braking"
                            )

            if not leader_brake_triggered and sim_time >= args.brake_time:
                leader_brake_triggered = True
                leader_brake_sim_time = sim_time
                leader_brake_duration = max(
                    args.step_length,
                    traci.vehicle.getSpeed(LEADER_ID) / args.leader_decel,
                )
                leader_stop_time = sim_time + leader_brake_duration
                traci.vehicle.slowDown(LEADER_ID, 0.0, leader_brake_duration)
                fallback_brake_time = sim_time + args.fallback_reaction_time
                if (
                    follower_brake_scheduled is None
                    or fallback_brake_time < follower_brake_scheduled
                ):
                    follower_brake_scheduled = fallback_brake_time
                    fallback_used = True
                print(
                    f"[event] t={sim_time:.1f}s leader emergency braking "
                    f"at {args.leader_decel:.1f} m/s²"
                )

            if (
                leader_brake_triggered
                and leader_stop_time is not None
                and sim_time >= leader_stop_time
            ):
                traci.vehicle.setSpeed(LEADER_ID, 0.0)

            if not follower_brake_triggered:
                traci.vehicle.setSpeed(FOLLOWER_ID, args.cruise_speed)

            if (
                not follower_brake_triggered
                and follower_brake_scheduled is not None
                and sim_time >= follower_brake_scheduled
            ):
                follower_brake_triggered = True
                follower_brake_sim_time = sim_time
                current_speed = max(0.0, traci.vehicle.getSpeed(FOLLOWER_ID))
                follower_brake_duration = max(
                    args.step_length,
                    current_speed / args.follower_decel,
                )
                follower_stop_time = sim_time + follower_brake_duration
                traci.vehicle.slowDown(
                    FOLLOWER_ID,
                    0.0,
                    follower_brake_duration,
                )
                source = "MEC warning" if warning_received and not fallback_used else "fallback"
                print(
                    f"[event] t={sim_time:.1f}s follower braking from {source} "
                    f"at {args.follower_decel:.1f} m/s²"
                )

            if (
                follower_brake_triggered
                and follower_stop_time is not None
                and sim_time >= follower_stop_time
            ):
                traci.vehicle.setSpeed(FOLLOWER_ID, 0.0)

            current_collision_ids = collision_ids()
            if current_collision_ids:
                unseen = set(current_collision_ids) - seen_collisions
                if unseen:
                    print(
                        f"[event] t={sim_time:.1f}s collision detected: "
                        f"{', '.join(sorted(current_collision_ids))}"
                    )
                seen_collisions.update(current_collision_ids)

            leader = vehicle_state(LEADER_ID)
            follower = vehicle_state(FOLLOWER_ID)
            truth = local_ground_truth(
                leader,
                follower,
                leader_decel=args.leader_decel,
                follower_decel=args.follower_decel,
            )
            state = event_state(
                leader_braking=leader_brake_triggered,
                warning_received=warning_received,
                follower_braking=follower_brake_triggered,
                collision=bool(current_collision_ids),
            )

            if mec_client and sim_time + 1e-9 >= next_send_time:
                client_send_ts = time.time()
                payload = {
                    "scenario": args.scenario,
                    "ue_id": args.ue_id,
                    "sim_time": sim_time,
                    "client_send_ts": client_send_ts,
                    "vehicles": [leader, follower],
                    "run_id": run_id,
                    "sequence": sequence,
                    "event_state": state,
                }
                mec_client.submit(
                    sequence=sequence,
                    sim_time=sim_time,
                    client_send_ts=client_send_ts,
                    payload=payload,
                )
                sequence += 1
                next_send_time += 1.0 / args.send_rate
                if next_send_time <= sim_time:
                    next_send_time = sim_time + 1.0 / args.send_rate

            row = {
                "run_id": run_id,
                "mode": args.mode,
                "sim_time": sim_time,
                "event_state": state,
                "leader_speed": leader["speed"],
                "leader_accel": leader["accel"],
                "leader_lane_pos": leader["lane_pos"],
                "follower_speed": follower["speed"],
                "follower_accel": follower["accel"],
                "follower_lane_pos": follower["lane_pos"],
                "gap": truth["gap"],
                "relative_speed": truth["relative_speed"],
                "ttc": truth["ttc"],
                "stopping_margin": truth["stopping_margin"],
                "warning_received": warning_received,
                "warning_sequence": warning_sequence,
                "warning_received_sim_time": warning_received_sim_time,
                "follower_brake_scheduled": follower_brake_scheduled,
                "follower_braking": follower_brake_triggered,
                "fallback_used": fallback_used,
                "collision": bool(current_collision_ids),
                "mec_rtt_ms": latest_network.get("rtt_ms", ""),
                "mec_risk_level": latest_network.get("risk_level", ""),
                "mec_warning": latest_network.get("warning", ""),
            }
            rows.append(row)

            whole_tenth = round(sim_time * 10)
            if whole_tenth % 10 == 0:
                ttc_text = (
                    f"{truth['ttc']:.2f}s"
                    if math.isfinite(truth["ttc"])
                    else "inf"
                )
                print(
                    f"[state] t={sim_time:>4.1f}s gap={truth['gap']:>6.2f}m "
                    f"TTC={ttc_text:>7} state={state}"
                )

    finally:
        if started:
            traci.close()
        if mec_client:
            trailing_results = mec_client.close()

    if mec_client:
        for result in trailing_results:
            network_rows.append(network_row(result, args.end_time))
        dropped_send_frames += mec_client.dropped_frames

    finite_gaps = [
        float(row["gap"])
        for row in rows
        if isinstance(row.get("gap"), (int, float))
        and math.isfinite(float(row["gap"]))
    ]
    finite_ttcs = [
        float(row["ttc"])
        for row in rows
        if isinstance(row.get("ttc"), (int, float))
        and math.isfinite(float(row["ttc"]))
    ]
    successful_posts = sum(bool(row.get("ok")) for row in network_rows)
    failed_posts = len(network_rows) - successful_posts
    collision = bool(seen_collisions) or any(bool(row["collision"]) for row in rows)

    summary = {
        "run_id": run_id,
        "scenario": args.scenario,
        "mode": args.mode,
        "outcome": "collision" if collision else "collision_avoided",
        "collision": collision,
        "colliding_vehicle_ids": sorted(seen_collisions),
        "minimum_gap_m": finite_or_none(min(finite_gaps)) if finite_gaps else None,
        "minimum_ttc_s": finite_or_none(min(finite_ttcs)) if finite_ttcs else None,
        "leader_brake_sim_time": leader_brake_sim_time,
        "warning_received": warning_received,
        "warning_applied": warning_received and not fallback_used,
        "warning_sequence": warning_sequence,
        "warning_report_sim_time": warning_report_sim_time,
        "warning_received_sim_time": warning_received_sim_time,
        "warning_received_wall_ts": warning_received_wall_ts,
        "follower_brake_sim_time": follower_brake_sim_time,
        "warning_to_brake_s": (
            follower_brake_sim_time - warning_received_sim_time
            if follower_brake_sim_time is not None
            and warning_received_sim_time is not None
            else None
        ),
        "fallback_used": fallback_used,
        "reports_submitted": sequence,
        "reports_completed": len(network_rows),
        "reports_ok": successful_posts,
        "reports_failed": failed_posts,
        "reports_dropped": dropped_send_frames,
        "config": {
            "step_length": args.step_length,
            "end_time": args.end_time,
            "send_rate": args.send_rate,
            "brake_time": args.brake_time,
            "cruise_speed": args.cruise_speed,
            "leader_decel": args.leader_decel,
            "follower_decel": args.follower_decel,
            "warning_reaction_time": args.warning_reaction_time,
            "fallback_reaction_time": args.fallback_reaction_time,
            "max_response_age": args.max_response_age,
            "mec_url": args.mec_url if args.mode == "mec" else None,
            "ue_interface": args.ue_interface if args.mode == "mec" else None,
        },
        "artifacts": {
            "step_csv": str(step_csv),
            "network_csv": str(network_csv) if network_rows else None,
            "fcd_xml": f"{prefix}_fcd.xml",
            "tripinfo_xml": f"{prefix}_tripinfo.xml",
            "collision_xml": f"{prefix}_collisions.xml",
        },
    }

    write_csv(step_csv, rows)
    write_csv(network_csv, network_rows)
    summary_json.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(
        f"[result] {summary['outcome']} min_gap={summary['minimum_gap_m']!r} "
        f"warning={warning_received}"
    )
    print(f"[result] summary={summary_json}")
    return summary


def main() -> int:
    args = parse_args()
    try:
        run(args)
    except (RuntimeError, OSError) as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
