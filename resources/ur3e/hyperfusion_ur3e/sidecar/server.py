#!/usr/bin/env python3
"""
UR3e HTTP server for HyperFusion (WSL sidecar).

Endpoints:
  GET  /health      -> sidecar + robot connection status
  POST /connect     -> connect to UR controller
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


def get_bridge(args: argparse.Namespace) -> Ur3eRosBridge:
    global _bridge
    with _bridge_lock:
        if _bridge is None:
            _bridge = Ur3eRosBridge(
                robot_ip=args.robot_ip,
                dashboard_port=args.dashboard_port,
                rtde_port=args.rtde_port,
                max_linear_speed_m_per_s=args.max_linear_speed,
                max_linear_accel_m_per_s2=args.max_linear_accel,
                ros_distro=args.ros_distro,
                ur_type=args.ur_type,
                use_mock_hardware=args.use_mock_hardware,
            )
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

    threading.Thread(target=_run, daemon=True, name="ur3e-prestart").start()


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()

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
