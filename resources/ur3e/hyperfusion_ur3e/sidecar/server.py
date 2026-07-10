#!/usr/bin/env python3
"""
UR3e HTTP server for HyperFusion (WSL sidecar).

Endpoints:
  GET  /health      -> sidecar + robot connection status
  POST /connect     -> connect to UR controller (blocking)
  POST /connect/start -> begin async connect (GUI)
  GET  /connect/status -> poll async connect progress
  POST /connect/cancel -> cancel async connect
  POST /disconnect  -> safe disconnect
  GET  /pose        -> current TCP pose [x,y,z,rx,ry,rz]
  GET  /joints      -> current joint names + positions (rad)
  POST /move_l      -> linear move in TCP frame
  POST /move_j      -> joint-space move
  POST /stop        -> halt motion
  POST /shutdown    -> stop server
"""
from __future__ import annotations

import argparse
import sys
import threading
from http.server import ThreadingHTTPServer
from pathlib import Path
from typing import Optional

from hyperfusion_ur3e.bridge.ros_bridge import Ur3eRosBridge
from hyperfusion_ur3e.sidecar.config import DEFAULT_CONFIG, load_yaml_config
from hyperfusion_ur3e.sidecar.http_handler import SidecarHttpHandler

_bridge: Optional[Ur3eRosBridge] = None
_bridge_lock = threading.Lock()


def _parse_initial_joint_deg(raw: str) -> list[float]:
    parts = [p.strip() for p in str(raw).split(",") if p.strip()]
    if len(parts) != 6:
        raise ValueError("initial_joint_deg must contain 6 comma-separated degree values.")
    return [float(p) for p in parts]


def get_bridge(args: argparse.Namespace) -> Ur3eRosBridge:
    global _bridge
    with _bridge_lock:
        if _bridge is None:
            _bridge = Ur3eRosBridge(
                robot_ip=args.robot_ip,
                reverse_ip=args.reverse_ip,
                dashboard_port=args.dashboard_port,
                rtde_port=args.rtde_port,
                max_linear_speed_m_per_s=args.max_linear_speed,
                max_linear_accel_m_per_s2=args.max_linear_accel,
                ros_distro=args.ros_distro,
                ur_type=args.ur_type,
                use_mock_hardware=args.use_mock_hardware,
                initial_joint_deg=args.initial_joint_deg,
                ceiling_mount_height_m=args.ceiling_mount_height_mm / 1000.0,
                workspace_boundary_enabled=args.workspace_boundary_enabled,
                workspace_length_m=args.workspace_length_mm / 1000.0,
                workspace_width_m=args.workspace_width_mm / 1000.0,
            )
            try:
                _bridge.start_workspace_boundary_sync()
            except Exception as exc:
                sys.stderr.write(f"UR3e sidecar: workspace boundary keepalive start failed: {exc}\n")
        return _bridge


def build_arg_parser() -> argparse.ArgumentParser:
    cfg = load_yaml_config(DEFAULT_CONFIG)
    robot_cfg = cfg.get("robot", {}) if isinstance(cfg.get("robot"), dict) else {}
    motion_cfg = cfg.get("motion", {}) if isinstance(cfg.get("motion"), dict) else {}
    server_cfg = cfg.get("server", {}) if isinstance(cfg.get("server"), dict) else {}

    parser = argparse.ArgumentParser(description="UR3e HTTP server for HyperFusion")
    parser.add_argument("--host", default=str(server_cfg.get("host", "0.0.0.0")))
    parser.add_argument("--port", type=int, default=int(server_cfg.get("port", 8766)))
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--robot-ip", default=str(robot_cfg.get("ip", "192.168.0.10")))
    parser.add_argument(
        "--reverse-ip",
        default=str(robot_cfg.get("reverse_ip", "0.0.0.0")),
        help="PC LAN IP the robot reaches for External Control (script_sender port 50002).",
    )
    parser.add_argument("--dashboard-port", type=int, default=int(robot_cfg.get("dashboard_port", 29999)))
    parser.add_argument("--rtde-port", type=int, default=int(robot_cfg.get("rtde_port", 30004)))
    parser.add_argument(
        "--max-linear-speed",
        type=float,
        default=float(motion_cfg.get("max_linear_speed_m_per_s", 0.05)),
    )
    parser.add_argument(
        "--max-linear-accel",
        type=float,
        default=float(motion_cfg.get("max_linear_accel_m_per_s2", 0.3)),
    )
    parser.add_argument("--ros-distro", default=str(cfg.get("ros_distro", "jazzy")))
    parser.add_argument("--ur-type", default=str(cfg.get("ur_type", "ur3e")))
    parser.add_argument(
        "--use-mock-hardware",
        action=argparse.BooleanOptionalAction,
        default=bool(cfg.get("use_mock_hardware", True)),
        help="Use ur_robot_driver ros2_control MockSystem (simulation).",
    )
    parser.add_argument(
        "--prestart-driver",
        action=argparse.BooleanOptionalAction,
        default=bool(cfg.get("prestart_driver", False)),
        help="Warm up ur_robot_driver in background after HTTP server starts.",
    )
    home_default = cfg.get("home_joints_deg", [0, -150, 120, 0, 90, 0])
    if isinstance(home_default, (list, tuple)) and len(home_default) == 6:
        home_default_str = ",".join(str(float(v)) for v in home_default)
    else:
        home_default_str = "0,-150,120,0,90,0"
    parser.add_argument(
        "--initial-joint-deg",
        default=home_default_str,
        help="Mock-only startup pose (degrees); HyperFusion passes home_joints_deg from hyperfusion.cfg.",
    )
    parser.add_argument(
        "--ceiling-mount-height-mm",
        type=float,
        default=float(cfg.get("ceiling_mount_height_m", 0.65)) * 1000.0,
        help="Ceiling mount height in mm (matches workspace_height_mm / Z=0 tray floor).",
    )
    parser.add_argument(
        "--workspace-boundary-enabled",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Publish workspace collision box into MoveIt/RViz.",
    )
    parser.add_argument(
        "--workspace-length-mm",
        type=float,
        default=600.0,
        help="Workspace tray length in mm (X axis).",
    )
    parser.add_argument(
        "--workspace-width-mm",
        type=float,
        default=600.0,
        help="Workspace tray width in mm (Y axis).",
    )
    return parser


def _prestart_driver_async(args: argparse.Namespace) -> None:
    def _run() -> None:
        try:
            sys.stderr.write("UR3e sidecar: prestarting ROS driver in background…\n")
            result = get_bridge(args).warm_driver()
            sys.stderr.write(
                f"UR3e sidecar: prestart driver launch requested ({result.get('driver', 'ok')})\n"
            )
        except Exception as exc:
            sys.stderr.write(f"UR3e sidecar: prestart failed: {exc}\n")
            try:
                bridge = get_bridge(args)
                with bridge._lock:
                    if bridge._connecting:
                        return
                    if bridge._driver is not None:
                        bridge._driver.stop()
                        bridge._driver = None
            except Exception:
                pass

    threading.Thread(target=_run, daemon=True, name="ur3e-prestart").start()


def main() -> None:
    import os

    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

    parser = build_arg_parser()
    args = parser.parse_args()
    os.environ["HYPERFUSION_USE_MOCK_HARDWARE"] = (
        "true" if args.use_mock_hardware else "false"
    )
    args.initial_joint_deg = _parse_initial_joint_deg(args.initial_joint_deg)

    bridge_supplier = lambda: get_bridge(args)
    SidecarHttpHandler.get_bridge = staticmethod(bridge_supplier)
    httpd = ThreadingHTTPServer((args.host, args.port), SidecarHttpHandler)
    mode = "simulation" if args.use_mock_hardware else "hardware"
    sys.stderr.write(
        f"UR3e sidecar listening on {args.host}:{args.port} "
        f"(mode={mode}, robot={args.robot_ip})\n"
    )
    if args.prestart_driver:
        _prestart_driver_async(args)
    httpd.serve_forever()


if __name__ == "__main__":
    main()
