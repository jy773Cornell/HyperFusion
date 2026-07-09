"""HTTP request handler for the UR3e sidecar API."""
from __future__ import annotations

import json
import sys
import threading
from http.server import BaseHTTPRequestHandler
from typing import Any, Callable, Dict

from hyperfusion_ur3e.bridge.ros_bridge import Ur3eRosBridge


class SidecarHttpHandler(BaseHTTPRequestHandler):
    """Routes REST calls to Ur3eRosBridge."""

    get_bridge: Callable[[], Ur3eRosBridge]

    def log_message(self, fmt: str, *values: Any) -> None:
        # Health polling is noisy; access logs are not useful in the HyperFusion log tab.
        return

    def _read_json(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length > 0 else b"{}"
        if not raw.strip():
            return {}
        data = json.loads(raw.decode("utf-8"))
        if not isinstance(data, dict):
            raise ValueError("JSON body must be an object")
        return data

    def _send_json(self, code: int, payload: Dict[str, Any]) -> None:
        data = json.dumps(payload).encode("utf-8")
        try:
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        except BrokenPipeError:
            pass

    def do_GET(self) -> None:
        path = self.path.rstrip("/")
        bridge = self.get_bridge()

        if path == "/health":
            status = bridge.status()
            self._send_json(
                200,
                {
                    "status": "ok",
                    "use_mock_hardware": bridge.use_mock_hardware,
                    "robot_connected": status.connected,
                    "driver_state": status.driver_state,
                    "robot_ip": status.robot_ip,
                    "joint_states_ok": bridge.joint_states_ok(),
                    "fault": status.fault or None,
                },
            )
            return

        if path == "/pose":
            try:
                self._send_json(200, bridge.get_pose())
            except Exception as exc:
                self._send_json(409, {"ok": False, "error": str(exc)})
            return

        if path == "/joints":
            try:
                self._send_json(200, bridge.get_joints())
            except Exception as exc:
                self._send_json(409, {"ok": False, "error": str(exc)})
            return

        self._send_json(404, {"ok": False, "error": "not found"})

    def do_POST(self) -> None:
        path = self.path.rstrip("/")
        bridge = self.get_bridge()

        if path == "/shutdown":
            self._send_json(200, {"ok": True, "shutting_down": True})
            threading.Thread(target=self._shutdown_server, daemon=True).start()
            return

        try:
            body = self._read_json()
        except Exception as exc:
            self._send_json(400, {"ok": False, "error": str(exc)})
            return

        if path == "/connect":
            if "ip" in body:
                bridge.robot_ip = str(body["ip"])
            try:
                self._send_json(200, bridge.connect())
            except Exception as exc:
                self._send_json(500, {"ok": False, "error": str(exc)})
            return

        if path == "/disconnect":
            self._send_json(200, bridge.disconnect())
            return

        if path == "/move_l":
            pose = body.get("pose")
            if not isinstance(pose, list) or len(pose) != 6:
                self._send_json(400, {"ok": False, "error": "pose must be a list of 6 floats"})
                return
            try:
                result = bridge.move_l(
                    pose,
                    speed=body.get("speed"),
                    accel=body.get("accel"),
                    wait=bool(body.get("wait", True)),
                )
                self._send_json(200, result)
            except Exception as exc:
                self._send_json(409, {"ok": False, "error": str(exc)})
            return

        if path == "/move_j":
            positions = body.get("positions")
            if not isinstance(positions, list) or len(positions) != 6:
                self._send_json(
                    400,
                    {"ok": False, "error": "positions must be a list of 6 floats (radians)"},
                )
                return
            try:
                result = bridge.move_j(positions, wait=bool(body.get("wait", True)))
                self._send_json(200, result)
            except Exception as exc:
                self._send_json(409, {"ok": False, "error": str(exc)})
            return

        if path == "/stop":
            self._send_json(200, bridge.stop())
            return

        self._send_json(404, {"ok": False, "error": "not found"})

    def _shutdown_server(self) -> None:
        try:
            self.get_bridge().shutdown()
        except Exception:
            pass
        self.server.shutdown()
