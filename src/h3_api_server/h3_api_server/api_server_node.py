#!/usr/bin/env python3

import math
import os
import threading
import asyncio
import glob
import time
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# ─────────────────────────── helpers ──────────────────────────────────────────

def _quat_to_yaw(x, y, z, w) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


# ─────────────────────────── Pydantic models ──────────────────────────────────

class MoveGoalRequest(BaseModel):
    type: str = "standard"          # standard | charge | rotate
    target_x: Optional[float] = None
    target_y: Optional[float] = None
    target_z: Optional[float] = 0.0
    target_ori: Optional[float] = None   # yaw in degrees
    target_accuracy: Optional[float] = 0.1
    use_target_zone: Optional[bool] = False
    approach_speed_limit: Optional[float] = None
    creator: Optional[str] = "api"


class PatchMoveRequest(BaseModel):
    state: str   # cancelled | paused


class PoseRequest(BaseModel):
    x: float
    y: float
    z: float = 0.0
    orientation: float = 0.0   # yaw in degrees


class TwistRequest(BaseModel):
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


class MapRequest(BaseModel):
    map_id: str


class RotateRequest(BaseModel):
    angle: float                        # target yaw in degrees
    angular_speed: Optional[float] = 0.5


# ─────────────────────────── shared node state ────────────────────────────────

class RobotState:
    def __init__(self):
        self.pose = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
        self.speed = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}
        self.battery = {"percentage": 1.0, "voltage": 24.0, "current": 0.0, "status": "unknown"}
        self.move_state = "idle"        # idle | moving | cancelled | completed | failed
        self.emergency_stop = False
        self.tray_open = False
        self.current_map = "unknown"
        self.nav_goal_handle = None
        self._lock = threading.Lock()


# ─────────────────────────── WebSocket manager ────────────────────────────────

class ConnectionManager:
    def __init__(self):
        self._clients: dict[str, list[WebSocket]] = {}
        self._lock = asyncio.Lock()

    async def connect(self, channel: str, ws: WebSocket):
        await ws.accept()
        async with self._lock:
            self._clients.setdefault(channel, []).append(ws)

    async def disconnect(self, channel: str, ws: WebSocket):
        async with self._lock:
            lst = self._clients.get(channel, [])
            if ws in lst:
                lst.remove(ws)

    async def broadcast(self, channel: str, data: dict):
        import json as _json
        msg = _json.dumps(data)
        async with self._lock:
            dead = []
            for ws in self._clients.get(channel, []):
                try:
                    await ws.send_text(msg)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self._clients[channel].remove(ws)


# ─────────────────────────── globals (set in main) ────────────────────────────

_node: "H3ApiServerNode" = None
_state = RobotState()
_ws_manager = ConnectionManager()

# ─────────────────────────── FastAPI app ──────────────────────────────────────

app = FastAPI(
    title="H3 Robot API Server",
    description="REST + WebSocket bridge for H3 robot control via ROS2/Nav2",
    version="1.0.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ── /robot/status ──────────────────────────────────────────────────────────────

@app.get("/robot/status", tags=["system"])
async def get_robot_status():
    with _state._lock:
        return {
            "pose": dict(_state.pose),
            "speed": dict(_state.speed),
            "battery": dict(_state.battery),
            "move_state": _state.move_state,
            "emergency_stop": _state.emergency_stop,
            "tray_open": _state.tray_open,
            "current_map": _state.current_map,
        }


# ── /chassis/moves ─────────────────────────────────────────────────────────────

@app.post("/chassis/moves", tags=["chassis"])
async def create_move(req: MoveGoalRequest):
    if _state.emergency_stop:
        raise HTTPException(status_code=409, detail="Emergency stop is active")
    if _state.move_state == "moving":
        raise HTTPException(status_code=409, detail="Robot is already moving")

    move_type = req.type.lower()

    if move_type == "charge":
        ok = _node.send_nav_goal_to_charge()
        if not ok:
            raise HTTPException(status_code=503, detail="Nav2 action server not available")
        return {"status": "accepted", "type": "charge"}

    if move_type == "standard":
        if req.target_x is None or req.target_y is None:
            raise HTTPException(status_code=422, detail="target_x and target_y are required")
        yaw_rad = _deg2rad(req.target_ori) if req.target_ori is not None else 0.0
        ok = _node.send_nav_goal(req.target_x, req.target_y, yaw_rad)
        if not ok:
            raise HTTPException(status_code=503, detail="Nav2 action server not available")
        return {
            "status": "accepted",
            "type": "standard",
            "goal": {"x": req.target_x, "y": req.target_y, "yaw_deg": req.target_ori},
        }

    if move_type == "rotate":
        if req.target_ori is None:
            raise HTTPException(status_code=422, detail="target_ori (yaw in degrees) is required")
        _node.start_rotate(req.target_ori, req.approach_speed_limit or 0.5)
        return {"status": "accepted", "type": "rotate", "target_yaw_deg": req.target_ori}

    raise HTTPException(status_code=422, detail=f"Unknown move type: {req.type}")


@app.get("/chassis/moves/current", tags=["chassis"])
async def get_current_move():
    with _state._lock:
        return {"move_state": _state.move_state, "pose": dict(_state.pose)}


@app.patch("/chassis/moves/current", tags=["chassis"])
async def patch_move(req: PatchMoveRequest):
    if req.state == "cancelled":
        _node.cancel_navigation()
        return {"status": "cancelled"}
    raise HTTPException(status_code=422, detail=f"Unsupported state: {req.state}")


# ── /chassis/pose ──────────────────────────────────────────────────────────────

@app.get("/chassis/pose", tags=["chassis"])
async def get_pose():
    with _state._lock:
        return dict(_state.pose)


@app.post("/chassis/pose", tags=["chassis"])
async def set_pose(req: PoseRequest):
    _node.publish_initial_pose(req.x, req.y, _deg2rad(req.orientation))
    with _state._lock:
        _state.pose = {"x": req.x, "y": req.y, "yaw_deg": req.orientation}
    return {"status": "ok", "pose": {"x": req.x, "y": req.y, "yaw_deg": req.orientation}}


# ── /chassis/twist ─────────────────────────────────────────────────────────────

@app.post("/chassis/twist", tags=["chassis"])
async def set_twist(req: TwistRequest):
    if _state.emergency_stop:
        raise HTTPException(status_code=409, detail="Emergency stop is active")
    _node.publish_twist(req.linear_x, req.linear_y, req.angular_z)
    return {"status": "ok", "twist": req.model_dump()}


@app.delete("/chassis/twist", tags=["chassis"])
async def stop_twist():
    _node.publish_twist(0.0, 0.0, 0.0)
    return {"status": "stopped"}


# ── /chassis/rotate ────────────────────────────────────────────────────────────

@app.post("/chassis/rotate", tags=["chassis"])
async def rotate(req: RotateRequest):
    if _state.emergency_stop:
        raise HTTPException(status_code=409, detail="Emergency stop is active")
    _node.start_rotate(req.angle, req.angular_speed or 0.5)
    return {"status": "accepted", "target_yaw_deg": req.angle}
    return {"status": "accepted", "target_yaw_deg": req.angle, "detail": result}


# ── /chassis/current-map ───────────────────────────────────────────────────────

@app.post("/chassis/current-map", tags=["maps"])
async def change_map(req: MapRequest):
    with _state._lock:
        _state.current_map = req.map_id
    _node.get_logger().info(f"Map changed to: {req.map_id}")
    return {"status": "ok", "current_map": req.map_id}


# ── /maps/ ─────────────────────────────────────────────────────────────────────

@app.get("/maps/", tags=["maps"])
async def list_maps():
    maps_dir = _node.get_parameter("maps_dir").value
    maps = []
    if maps_dir and os.path.isdir(maps_dir):
        for f in sorted(glob.glob(os.path.join(maps_dir, "*.yaml"))):
            map_id = os.path.splitext(os.path.basename(f))[0]
            maps.append({"map_id": map_id, "path": f})
    return {"maps": maps, "count": len(maps)}


# ── /tray ──────────────────────────────────────────────────────────────────────

@app.post("/tray/open", tags=["tray"])
async def open_tray():
    with _state._lock:
        _state.tray_open = True
    _node.publish_tray_command(True)
    return {"status": "ok", "tray": "open"}


@app.post("/tray/close", tags=["tray"])
async def close_tray():
    with _state._lock:
        _state.tray_open = False
    _node.publish_tray_command(False)
    return {"status": "ok", "tray": "closed"}


@app.get("/tray/status", tags=["tray"])
async def tray_status():
    with _state._lock:
        return {"tray_open": _state.tray_open}


# ── /emergency-stop ────────────────────────────────────────────────────────────

@app.post("/emergency-stop", tags=["safety"])
async def trigger_estop():
    with _state._lock:
        _state.emergency_stop = True
    _node.publish_twist(0.0, 0.0, 0.0)
    _node.cancel_navigation()
    return {"status": "ok", "emergency_stop": True}


@app.post("/emergency-stop/release", tags=["safety"])
async def release_estop():
    with _state._lock:
        _state.emergency_stop = False
    _node.get_logger().info("Emergency stop released")
    return {"status": "ok", "emergency_stop": False}


@app.get("/emergency-stop/status", tags=["safety"])
async def estop_status():
    with _state._lock:
        return {"emergency_stop": _state.emergency_stop}


# ── /system ────────────────────────────────────────────────────────────────────

@app.post("/system/restart", tags=["system"])
async def restart_node():
    _node.get_logger().warn("Restart requested via API – shutting down ROS node")
    threading.Thread(target=_graceful_restart, daemon=True).start()
    return {"status": "restarting"}


def _graceful_restart():
    time.sleep(0.5)
    rclpy.shutdown()


# ── WebSocket endpoints ────────────────────────────────────────────────────────

@app.websocket("/ws/pose")
async def ws_pose(ws: WebSocket):
    await _ws_manager.connect("pose", ws)
    try:
        while True:
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass
    finally:
        await _ws_manager.disconnect("pose", ws)


@app.websocket("/ws/battery")
async def ws_battery(ws: WebSocket):
    await _ws_manager.connect("battery", ws)
    try:
        while True:
            await asyncio.sleep(1.0)
    except WebSocketDisconnect:
        pass
    finally:
        await _ws_manager.disconnect("battery", ws)


@app.websocket("/ws/speed")
async def ws_speed(ws: WebSocket):
    await _ws_manager.connect("speed", ws)
    try:
        while True:
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass
    finally:
        await _ws_manager.disconnect("speed", ws)


@app.websocket("/ws/status")
async def ws_status(ws: WebSocket):
    await _ws_manager.connect("status", ws)
    try:
        while True:
            await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        pass
    finally:
        await _ws_manager.disconnect("status", ws)


# ─────────────────────────── ROS2 Node ────────────────────────────────────────

class H3ApiServerNode(Node):
    def __init__(self):
        super().__init__("h3_api_server")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8090)
        self.declare_parameter("maps_dir", "")
        self.declare_parameter("charge_x", 0.0)
        self.declare_parameter("charge_y", 0.0)
        self.declare_parameter("charge_yaw_deg", 0.0)

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "initialpose", 10
        )
        self._tray_pub = self.create_publisher(Bool, "tray/command", 10)

        self._odom_sub = self.create_subscription(
            Odometry, "odom", self._odom_cb, qos_sensor
        )
        self._cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self._cmd_vel_cb, 10
        )
        self._battery_sub = self.create_subscription(
            BatteryState, "battery_state", self._battery_cb, qos_sensor
        )

        # asyncio event loop for Nav2 and WebSocket broadcasts
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._loop.run_forever, daemon=True
        )
        self._loop_thread.start()

        # periodic broadcast timer
        self.create_timer(0.2, self._broadcast_pose_and_speed)
        self.create_timer(2.0, self._broadcast_battery)
        self.create_timer(0.5, self._broadcast_status)

        # start uvicorn in a thread
        host = self.get_parameter("host").value
        port = self.get_parameter("port").value
        threading.Thread(
            target=uvicorn.run,
            kwargs={"app": app, "host": host, "port": port, "log_level": "warning"},
            daemon=True,
        ).start()

        self.get_logger().info(
            f"H3 API Server started at http://{host}:{port}  (docs: /docs)"
        )

    # ── ROS callbacks ──────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        with _state._lock:
            _state.pose = {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "yaw_deg": _rad2deg(yaw),
            }

    def _cmd_vel_cb(self, msg: Twist):
        with _state._lock:
            _state.speed = {
                "linear_x": msg.linear.x,
                "linear_y": msg.linear.y,
                "angular_z": msg.angular.z,
            }

    def _battery_cb(self, msg: BatteryState):
        status_map = {
            BatteryState.POWER_SUPPLY_STATUS_CHARGING: "charging",
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING: "discharging",
            BatteryState.POWER_SUPPLY_STATUS_FULL: "full",
            BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING: "not_charging",
        }
        with _state._lock:
            _state.battery = {
                "percentage": round(msg.percentage * 100, 1),
                "voltage": round(msg.voltage, 2),
                "current": round(msg.current, 2),
                "status": status_map.get(msg.power_supply_status, "unknown"),
            }

    # ── ROS publishers ─────────────────────────────────────────────────────────

    def publish_twist(self, lx: float, ly: float, az: float):
        msg = Twist()
        msg.linear.x = float(lx)
        msg.linear.y = float(ly)
        msg.angular.z = float(az)
        self._cmd_vel_pub.publish(msg)

    def publish_initial_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self._init_pose_pub.publish(msg)

    def publish_tray_command(self, open_tray: bool):
        msg = Bool()
        msg.data = open_tray
        self._tray_pub.publish(msg)

    # ── Nav2 helpers (callback-based, no cross-loop await) ────────────────────

    def _build_nav_goal(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        return goal

    def send_nav_goal(self, x: float, y: float, yaw: float) -> bool:
        """Send a Nav2 goal. Returns False if action server is unavailable.
        move_state is updated via callbacks driven by rclpy executor."""
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Nav2 action server not available")
            return False

        goal = self._build_nav_goal(x, y, yaw)

        with _state._lock:
            _state.move_state = "moving"

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)
        return True

    def send_nav_goal_to_charge(self) -> bool:
        cx = self.get_parameter("charge_x").value
        cy = self.get_parameter("charge_y").value
        cyaw = _deg2rad(self.get_parameter("charge_yaw_deg").value)
        return self.send_nav_goal(cx, cy, cyaw)

    def _on_goal_response(self, future):
        """Called by rclpy executor when Nav2 accepts/rejects the goal."""
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Nav2 rejected goal")
            with _state._lock:
                _state.move_state = "failed"
                _state.nav_goal_handle = None
            return

        self.get_logger().info("Nav2 accepted goal — waiting for result")
        with _state._lock:
            _state.nav_goal_handle = handle

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        """Called by rclpy executor when Nav2 finishes (success/fail/cancel)."""
        from action_msgs.msg import GoalStatus
        try:
            result = future.result()
            status = result.status
            with _state._lock:
                if _state.move_state != "cancelled":
                    if status == GoalStatus.STATUS_SUCCEEDED:
                        _state.move_state = "completed"
                        self.get_logger().info("Navigation completed successfully")
                    else:
                        _state.move_state = "failed"
                        self.get_logger().warn(f"Navigation ended with status {status}")
                _state.nav_goal_handle = None
        except Exception as e:
            self.get_logger().error(f"Nav2 result error: {e}")
            with _state._lock:
                _state.move_state = "failed"
                _state.nav_goal_handle = None

    def cancel_navigation(self):
        """Cancel any active Nav2 goal. Safe to call from any thread."""
        with _state._lock:
            handle = _state.nav_goal_handle
            _state.move_state = "cancelled"
            _state.nav_goal_handle = None

        if handle is not None:
            cancel_future = handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda _: self.get_logger().info("Navigation cancelled")
            )

    def start_rotate(self, target_deg: float, angular_speed: float):
        """Fire-and-forget rotation; runs on self._loop."""
        asyncio.run_coroutine_threadsafe(
            self._rotate_coroutine(target_deg, angular_speed),
            self._loop,
        )

    async def _rotate_coroutine(self, target_deg: float, angular_speed: float):
        with _state._lock:
            _state.move_state = "moving"

        target_yaw = _deg2rad(target_deg)
        with _state._lock:
            cur_yaw = _deg2rad(_state.pose.get("yaw_deg", 0.0))

        diff = math.atan2(math.sin(target_yaw - cur_yaw), math.cos(target_yaw - cur_yaw))
        direction = 1.0 if diff >= 0 else -1.0
        speed = abs(angular_speed) * direction
        start = time.time()
        timeout = abs(diff) / max(abs(angular_speed), 0.01) + 2.0

        while True:
            with _state._lock:
                if _state.move_state == "cancelled":
                    break
                cur_yaw = _deg2rad(_state.pose.get("yaw_deg", 0.0))

            remaining = math.atan2(
                math.sin(target_yaw - cur_yaw), math.cos(target_yaw - cur_yaw)
            )
            if abs(remaining) < _deg2rad(2.0) or time.time() - start > timeout:
                break

            self.publish_twist(0.0, 0.0, speed)
            await asyncio.sleep(0.05)

        self.publish_twist(0.0, 0.0, 0.0)
        with _state._lock:
            if _state.move_state != "cancelled":
                _state.move_state = "completed"

    # ── periodic broadcasts ────────────────────────────────────────────────────

    def _broadcast_pose_and_speed(self):
        with _state._lock:
            pose = dict(_state.pose)
            speed = dict(_state.speed)
        asyncio.run_coroutine_threadsafe(
            _ws_manager.broadcast("pose", pose), self._loop
        )
        asyncio.run_coroutine_threadsafe(
            _ws_manager.broadcast("speed", speed), self._loop
        )

    def _broadcast_battery(self):
        with _state._lock:
            battery = dict(_state.battery)
        asyncio.run_coroutine_threadsafe(
            _ws_manager.broadcast("battery", battery), self._loop
        )

    def _broadcast_status(self):
        with _state._lock:
            status = {
                "pose": dict(_state.pose),
                "speed": dict(_state.speed),
                "battery": dict(_state.battery),
                "move_state": _state.move_state,
                "emergency_stop": _state.emergency_stop,
                "tray_open": _state.tray_open,
                "current_map": _state.current_map,
            }
        asyncio.run_coroutine_threadsafe(
            _ws_manager.broadcast("status", status), self._loop
        )


# ─────────────────────────── entry point ──────────────────────────────────────

def main(args=None):
    global _node
    rclpy.init(args=args)
    _node = H3ApiServerNode()
    try:
        rclpy.spin(_node)
    except KeyboardInterrupt:
        pass
    finally:
        _node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
