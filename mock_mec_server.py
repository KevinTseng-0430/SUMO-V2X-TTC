#!/usr/bin/env python3
"""Local MEC-compatible server for repeatable closed-loop verification."""

from __future__ import annotations

import argparse
import json
import math
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any


class RiskEvaluator:
    def __init__(self, ttc_threshold: float, delay_ms: float) -> None:
        self.ttc_threshold = ttc_threshold
        self.delay_ms = delay_ms

    def evaluate(self, payload: dict[str, Any]) -> dict[str, Any]:
        vehicles = payload.get("vehicles", [])
        if not isinstance(vehicles, list):
            vehicles = []
        by_id = {
            vehicle.get("vehicle_id"): vehicle
            for vehicle in vehicles
            if isinstance(vehicle, dict)
        }
        leader = by_id.get("leader_car")
        follower = by_id.get("follower_car")
        if leader is None or follower is None:
            return {
                "gap": None,
                "relative_speed": None,
                "ttc": None,
                "risk_level": "unknown",
                "warning": False,
                "leader_id": "",
                "follower_id": "",
            }

        same_lane = leader.get("lane_id") == follower.get("lane_id")
        gap = (
            float(leader["lane_pos"])
            - float(leader.get("length", 5.0))
            - float(follower["lane_pos"])
            if same_lane
            else math.nan
        )
        relative_speed = float(follower["speed"]) - float(leader["speed"])
        ttc = (
            gap / relative_speed
            if same_lane and gap >= 0.0 and relative_speed > 1e-6
            else math.inf
        )
        warning = math.isfinite(ttc) and ttc <= self.ttc_threshold
        if warning and ttc <= 2.0:
            risk_level = "high"
        elif warning:
            risk_level = "medium"
        elif math.isfinite(ttc):
            risk_level = "low"
        else:
            risk_level = "none"

        return {
            "gap": gap if math.isfinite(gap) else None,
            "relative_speed": relative_speed,
            "ttc": ttc if math.isfinite(ttc) else None,
            "risk_level": risk_level,
            "warning": warning,
            "leader_id": "leader_car",
            "follower_id": "follower_car",
        }


class MecHandler(BaseHTTPRequestHandler):
    evaluator: RiskEvaluator
    verbose: bool = False
    server_version = "MockMEC/1.0"

    def do_GET(self) -> None:
        if self.path != "/health":
            self.send_error(404)
            return
        self._send_json(200, {"ok": True, "service": "mock-mec"})

    def do_POST(self) -> None:
        server_recv_ts = time.time()
        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(content_length))
            if not isinstance(payload, dict):
                raise ValueError("request JSON must be an object")
        except (ValueError, json.JSONDecodeError) as exc:
            self._send_json(400, {"error": str(exc)})
            return

        if self.evaluator.delay_ms > 0:
            time.sleep(self.evaluator.delay_ms / 1000.0)
        result = self.evaluator.evaluate(payload)
        server_send_ts = time.time()
        response = {
            "server_recv_ts": server_recv_ts,
            "server_send_ts": server_send_ts,
            "proc_delay_ms": (server_send_ts - server_recv_ts) * 1000.0,
            "client_ip": self.client_address[0],
            "run_id": payload.get("run_id"),
            "sequence": payload.get("sequence"),
            "result": result,
        }
        self._send_json(200, response)

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def log_message(self, format_string: str, *args: Any) -> None:
        if self.verbose:
            super().log_message(format_string, *args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a local mock V2X MEC server.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18080)
    parser.add_argument("--ttc-threshold", type=float, default=4.0)
    parser.add_argument(
        "--delay-ms",
        type=float,
        default=10.0,
        help="Artificial server processing delay.",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.ttc_threshold <= 0 or args.delay_ms < 0:
        parser.error("threshold must be positive and delay cannot be negative")
    return args


def main() -> int:
    args = parse_args()
    MecHandler.evaluator = RiskEvaluator(args.ttc_threshold, args.delay_ms)
    MecHandler.verbose = args.verbose
    server = ThreadingHTTPServer((args.host, args.port), MecHandler)
    print(
        f"[mock-mec] listening on http://{args.host}:{args.port} "
        f"(TTC threshold={args.ttc_threshold:.1f}s, delay={args.delay_ms:.1f}ms)"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    print("[mock-mec] stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
