#!/usr/bin/env python3

import json
import uuid
import math
import threading
import time
import yaml
import numpy as np
from scipy.interpolate import CubicSpline

import paho.mqtt.client as mqtt

from elevator_robot.elevator_mqtt import ElevatorMqtt

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, FollowPath
from nav_msgs.msg import Path as PathMsg
from nav2_msgs.srv import LoadMap

# ============================================================
# GUI ROS2 MESSAGE
# ============================================================

from std_msgs.msg import String, Int32


# ============================================================
# CONFIG
# ============================================================

MQTT_BROKER = "10.11.50.162"
MQTT_PORT = 1883

REQUEST_TOPIC = "elevator/request"
STATUS_TOPIC = "elevator/status"

ROBOT_ID = "robot3"

CONFIG_FILE = "/home/khai/elevator_robot/config/elevator.yaml"
WAYPOINTS_FILE = "/home/khai/elevator_robot/config/elevator_waypoints.yaml"
RPP_CONTROLLER_ID = "ElevatorEntry"
RPP_NUM_PATH_POINTS = 50
POST_EXIT_SETTLE_TIME = 1.5

MAP_FILES = {
    1: "/home/khai/Map_turtlebot/map4.yaml",
    2: "/home/khai/Map_turtlebot/map5.yaml",
}

NAV_ACTION = "/robot3/navigate_to_pose"
MAP_SERVICE = "/robot3/map_server/load_map"


# ============================================================
# EXIT ELEVATOR CONFIG
# ============================================================

EXIT_DISTANCE_THRESHOLD = 1.50
EXIT_CONFIRM_TIME = 1.0
EXIT_DOOR_DELAY = 2.0
AMCL_STABILIZE_TIME = 5.0


# ============================================================
# ELEVATOR MANAGER
# ============================================================

class ElevatorManager(Node):

    def __init__(self):

        super().__init__("elevator_manager")

        # ----------------------------------------------------
        # Robot state
        # ----------------------------------------------------

        self.robot_floor = 1

        self.destination_floor = None
        self.destination_room = None

        self.trip_active = False
        self._task_lock = threading.Lock()
        self._cancel_event = threading.Event()
        self._shutdown_event = threading.Event()
        self._worker = None
        self._active_goal = None
        self._pending_goal = None
        self._pending_map = None
        self.task_state = "IDLE"
        self.current_step = "IDLE"
        self.last_error = None
        self.trip_id = ""

        self.robot_at_waiting_position = False
        self.robot_inside_elevator = False

        # ----------------------------------------------------
        # Robot position
        # ----------------------------------------------------

        self.robot_x = None
        self.robot_y = None
        self.amcl_pose_received_at = 0.0

        # ----------------------------------------------------
        # Publisher: initialize AMCL after changing floors
        # ----------------------------------------------------

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/robot3/initialpose",
            10
        )

        # ----------------------------------------------------
        # Subscribe AMCL pose
        # ----------------------------------------------------

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/robot3/amcl_pose",
            self.amcl_pose_callback,
            10
        )

        self.get_logger().info(
            "Đã subscribe /robot3/amcl_pose"
        )

        # ----------------------------------------------------
        # Elevator state
        # ----------------------------------------------------

        self.elevator_floor = None
        self.elevator_state = "UNKNOWN"


        # ----------------------------------------------------
        # Load YAML
        # ----------------------------------------------------

        try:

            with open(CONFIG_FILE, "r") as file:

                self.config = yaml.safe_load(file)

            self.get_logger().info(
                f"Đã đọc cấu hình: {CONFIG_FILE}"
            )

        except Exception as e:

            self.get_logger().error(
                f"Không thể đọc elevator.yaml: {e}"
            )

            raise

        # ----------------------------------------------------
        # Load RPP entry/exit paths (map4=floor_1, map5=floor_2)
        # ----------------------------------------------------
        self.rpp_paths = {}
        try:
            with open(WAYPOINTS_FILE, "r", encoding="utf-8") as file:
                waypoint_root = yaml.safe_load(file)

            elevator_paths = waypoint_root["elevator"]["maps"]
            map_to_floor = {"map4": 1, "map5": 2}

            for map_id, floor in map_to_floor.items():
                floor_key = f"floor_{floor}"
                approach = self.config[floor_key]["approach_pose"]
                approach_yaw = float(approach["yaw"])
                approach_point = (
                    float(approach["x"]),
                    float(approach["y"]),
                    math.sin(approach_yaw / 2.0),
                    math.cos(approach_yaw / 2.0),
                )

                map_paths = elevator_paths[map_id]
                entry = [tuple(float(v) for v in point)
                         for point in map_paths["entry_waypoints"]]
                exit_path = [tuple(float(v) for v in point)
                             for point in map_paths["exit_waypoints"]]

                if len(entry) < 2 or len(exit_path) < 2:
                    raise ValueError(f"{map_id}: entry/exit cần ít nhất 2 waypoint.")
                if any(len(point) != 4 for point in entry + exit_path):
                    raise ValueError(f"{map_id}: waypoint phải có dạng [x, y, qz, qw].")

                # Waypoint cuối của entry chính là điểm parking trong cabin.
                # Exit bắt đầu từ đúng điểm này (đường đi ngược entry).
                parking_point = entry[-1]

                if any(
                    abs(a - b) > 1e-4
                    for a, b in zip(parking_point, exit_path[0])
                ):
                    raise ValueError(
                        f"{map_id}: exit_waypoints[0] không trùng "
                        "entry_waypoints[-1] (parking pose)."
                    )

                self.rpp_paths[floor] = {
                    "entry": [approach_point] + entry,
                    "exit": exit_path,
                    "parking_pose": parking_point,
                }

            self.get_logger().info(
                f"Đã nạp quỹ đạo RPP entry/exit từ {WAYPOINTS_FILE}"
            )
        except Exception as error:
            self.get_logger().error(
                f"Không nạp được cấu hình RPP {WAYPOINTS_FILE}: {error}"
            )
            raise

        self.follow_path_client = ActionClient(
            self, FollowPath, "/robot3/follow_path"
        )

        # ----------------------------------------------------
        # Nav2 Action Client
        # ----------------------------------------------------

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            NAV_ACTION
        )

        self.get_logger().info(
            f"Đang chờ Nav2 action {NAV_ACTION}..."
        )

        if not self.nav_client.wait_for_server(
            timeout_sec=10.0
        ):

            self.get_logger().error(
                "Không tìm thấy Nav2 action server!"
            )

        else:

            self.get_logger().info(
                "Đã kết nối Nav2."
            )

        # ----------------------------------------------------
        # Map service
        # ----------------------------------------------------

        self.map_client = self.create_client(
            LoadMap,
            MAP_SERVICE
        )

        self.get_logger().info(
            f"Đang chờ map service {MAP_SERVICE}..."
        )

        if not self.map_client.wait_for_service(
            timeout_sec=10.0
        ):

            self.get_logger().warn(
                "Chưa tìm thấy map service."
            )

        else:

            self.get_logger().info(
                "Đã kết nối map service."
            )

        # ----------------------------------------------------
        # MQTT
        # ----------------------------------------------------

        self.mqtt_client = mqtt.Client()
        self.elevator_link = ElevatorMqtt(
            lambda payload: self.mqtt_client.publish(REQUEST_TOPIC, payload, qos=1, retain=False),
            ROBOT_ID,
        )
        self.mqtt_client.on_disconnect = self.mqtt_on_disconnect
        self.mqtt_client.on_subscribe = self.mqtt_on_subscribe

        self.mqtt_client.on_connect = (
            self.mqtt_on_connect
        )

        self.mqtt_client.on_message = (
            self.mqtt_on_message
        )

        try:

            self.mqtt_client.connect_async(
                MQTT_BROKER,
                MQTT_PORT,
                60
            )

            self.mqtt_client.loop_start()

            self.get_logger().info(
                f"Đang kết nối MQTT broker "
                f"{MQTT_BROKER}:{MQTT_PORT}"
            )

        except Exception as e:

            self.get_logger().error(
                f"Lỗi kết nối MQTT: {e}"
            )

        # ====================================================
        # GUI COMMUNICATION
        # ====================================================

        # ----------------------------------------------------
        # Manager -> GUI
        # ----------------------------------------------------

        self.gui_status_pub = self.create_publisher(
            String,
            "/elevator/manager_status",
            10
        )

        # ----------------------------------------------------
        # GUI -> Manager
        # ----------------------------------------------------

        self.gui_destination_sub = self.create_subscription(
            Int32,
            "/elevator/set_destination",
            self.gui_destination_callback,
            10
        )

        # ----------------------------------------------------
        # Publish trạng thái cho GUI mỗi 0.5 giây
        # ----------------------------------------------------

        self.gui_status_timer = self.create_timer(
            0.5,
            self.publish_gui_status
        )

        self.get_logger().info(
            "GUI interface ready:"
        )

        self.get_logger().info(
            "  /elevator/manager_status"
        )

        self.get_logger().info(
            "  /elevator/set_destination"
        )


    def _check_running(self):
        self.elevator_link.assert_guard()
        if self._shutdown_event.is_set() or self._cancel_event.is_set() or not rclpy.ok():
            raise RuntimeError("Nhiệm vụ đã bị hủy hoặc ROS đang dừng.")

    def _pause(self, seconds):
        self._check_running()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._cancel_event.wait(min(0.1, max(0.0, deadline - time.monotonic())))
            self._check_running()

    def _wait_future(self, future, timeout, label, interruptible=True):
        deadline = time.monotonic() + timeout
        while not future.done():
            if interruptible:
                self._check_running()
            if time.monotonic() >= deadline or not rclpy.ok():
                raise TimeoutError(f"Timeout chờ {label} ({timeout}s).")
            time.sleep(0.05)
        return future.result()

    def _cancel_active_goal(self):
        handle = self._active_goal
        if handle is None:
            return True
        try:
            # Wait for a terminal result, not just acknowledgement of the request.
            self._wait_future(handle.cancel_goal_async(), 5.0, "cancel goal", False)
            self._wait_future(handle.get_result_async(), 5.0, "goal kết thúc", False)
            self._active_goal = None
            return True
        except Exception as error:
            self.get_logger().error(f"Chưa xác nhận goal đã dừng: {error}")
            return False

    def _execute_action(self, client, goal, timeout, label):
        self._check_running()
        future = client.send_goal_async(goal)
        self._pending_goal = future
        try:
            handle = self._wait_future(future, 10.0, f"nhận goal {label}")
        except Exception:
            # A server can accept a goal after our response deadline. Keep it
            # tracked and cancel it when its handle eventually arrives.
            def cancel_late(done):
                try:
                    late = done.result()
                    if late is not None and late.accepted:
                        self._active_goal = late
                        late.cancel_goal_async()
                except Exception as error:
                    self.get_logger().error(f"Không đọc được goal đến muộn: {error}")
                finally:
                    self._pending_goal = None
            future.add_done_callback(cancel_late)
            raise
        self._pending_goal = None
        if handle is None or not handle.accepted:
            raise RuntimeError(f"Server từ chối {label}.")
        self._active_goal = handle
        try:
            result = self._wait_future(handle.get_result_async(), timeout, label)
            self._active_goal = None
            return result
        except Exception:
            self._cancel_active_goal()
            raise

    def submit_room(self, room):
        if self.get_room_info(room) is None:
            return False
        if not self._task_lock.acquire(blocking=False):
            self.get_logger().warning("Đang có thao tác khác; từ chối yêu cầu mới.")
            return False
        if self.task_state == "RECOVERY_REQUIRED" or self._shutdown_event.is_set():
            self._task_lock.release()
            self.get_logger().warning("Cần kiểm tra và xác nhận phục hồi trước chuyến mới.")
            return False
        self._cancel_event.clear()
        self.trip_active = True
        self.task_state = "RUNNING"
        self.current_step = "STARTING"
        self.trip_id = uuid.uuid4().hex
        self.last_error = None
        self.destination_room = int(room)
        self.destination_floor = self.get_room_info(room)[0]
        self._worker = threading.Thread(target=self._run_room, args=(room,), daemon=True)
        try:
            self._worker.start()
        except Exception:
            self.trip_active = False
            self.task_state = "RECOVERY_REQUIRED"
            self._task_lock.release()
            raise
        return True

    def _run_room(self, room):
        try:
            if not self._go_to_room(room):
                raise RuntimeError(f"Không hoàn thành bước {self.current_step}.")
            self._check_running()
            self.task_state = "COMPLETED"
            self.current_step = "COMPLETED"
        except Exception as error:
            self.last_error = str(error)
            self.task_state = "RECOVERY_REQUIRED"
            self._cancel_active_goal()
            self.get_logger().error(f"Cần phục hồi tại {self.current_step}: {error}")
        finally:
            self.elevator_link.end_guard()
            self.trip_active = False
            self.destination_floor = None
            self.destination_room = None
            self._task_lock.release()

    def cancel_trip(self):
        self._cancel_event.set()

    def recover(self):
        if not self._task_lock.acquire(blocking=False):
            self.get_logger().warning("Nhiệm vụ chưa kết thúc; chưa thể phục hồi.")
            return False
        try:
            if self._pending_map is not None and not self._pending_map.done():
                self.get_logger().warning("LoadMap chưa kết thúc; chưa thể tiếp tục.")
                return False
            if self._pending_goal is not None or not self._cancel_active_goal():
                self.get_logger().warning("Goal chưa xác nhận kết thúc; tiếp tục khóa nhiệm vụ.")
                return False
            if self._shutdown_event.is_set():
                return False
            # Operator explicitly confirms robot is outside, floor/map/pose agree.
            self.robot_inside_elevator = False
            self.robot_at_waiting_position = False
            self._cancel_event.clear()
            self.task_state = "IDLE"
            self.current_step = "IDLE"
            self.last_error = None
            return True
        finally:
            self._task_lock.release()

    def set_floor_manually(self, floor):
        if not self._task_lock.acquire(blocking=False):
            self.get_logger().warning("Không đổi map khi nhiệm vụ đang chạy.")
            return False
        try:
            if self._pending_map is not None and not self._pending_map.done():
                self.get_logger().warning("LoadMap chưa kết thúc; chưa thể tiếp tục.")
                return False
            if self._pending_goal is not None or not self._cancel_active_goal():
                return False
            self._cancel_event.clear()
            if not self.switch_map(floor):
                raise RuntimeError("Đổi map thủ công thất bại.")
            self.robot_floor = floor
            self.task_state = "RECOVERY_REQUIRED"
            self.last_error = "Đã đổi map thủ công; kiểm tra initial pose rồi xác nhận phục hồi."
            return True
        except Exception as error:
            self.task_state = "RECOVERY_REQUIRED"
            self.last_error = str(error)
            return False
        finally:
            self._task_lock.release()

    def stop_tasks(self):
        self._shutdown_event.set()
        self.cancel_trip()
        if self._worker is not None:
            self._worker.join(timeout=35.0)
        self._cancel_active_goal()
        if self._pending_goal is not None:
            self.get_logger().error("Còn yêu cầu goal chưa phản hồi khi thoát; cần kiểm tra Nav2.")

    # ========================================================
    # GUI -> MANAGER
    # ========================================================

    def gui_destination_callback(self, msg):

        destination_room = int(msg.data)

        self.get_logger().info(
            f"[GUI] Nhận yêu cầu đi tới phòng {destination_room}"
        )

        # ----------------------------------------------------
        # Kiểm tra phòng có tồn tại trong YAML
        # ----------------------------------------------------

        room_info = self.get_room_info(destination_room)

        if room_info is None:

            self.get_logger().warning(
                f"[GUI] Phòng không hợp lệ: {destination_room}"
            )

            return

        destination_floor, room_pose = room_info

        self.get_logger().info(
            f"[GUI] Phòng {destination_room} "
            f"thuộc tầng {destination_floor}"
        )

        # ----------------------------------------------------
        # Nếu đang có chuyến
        # ----------------------------------------------------

        if self.trip_active:

            self.get_logger().warning(
                "[GUI] Robot đang thực hiện chuyến."
            )

            return

        # ----------------------------------------------------
        # Chạy trip bằng thread riêng
        # ----------------------------------------------------

        self.submit_room(destination_room)


    # ========================================================
    # MANAGER -> GUI
    # ========================================================

    def publish_gui_status(self):

        data = {
            "trip_id": self.trip_id,
            "mqtt_connected": self.elevator_link.connected,
            "esp_online": self.elevator_link.healthy(),
            "task_state": self.task_state,
            "current_step": self.current_step,
            "last_error": self.last_error,
            "robot_floor": self.robot_floor,

            "robot_x": self.robot_x,
            "robot_y": self.robot_y,

            "robot_waiting": (
                self.robot_at_waiting_position
            ),

            "robot_inside": (
                self.robot_inside_elevator
            ),

            "elevator_floor": self.elevator_floor,

            "elevator_state": (
                self.elevator_state
            ),

            "trip_active": self.trip_active,

            "destination_floor": (
                self.destination_floor
            ),

            "destination_room": (
                self.destination_room
            )
        }

        msg = String()

        msg.data = json.dumps(data)

        self.gui_status_pub.publish(msg)


    # ========================================================
    # AMCL POSE
    # ========================================================

    def amcl_pose_callback(self, msg):

        self.robot_x = (
            msg.pose.pose.position.x
        )

        self.robot_y = (
            msg.pose.pose.position.y
        )

        self.amcl_pose_received_at = time.monotonic()


    # ========================================================
    # MQTT
    # ========================================================

    def mqtt_on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            # Subscribe before enabling requests; never block the Paho callback.
            client.subscribe(STATUS_TOPIC, qos=1)
        else:
            self.elevator_link.connection(False)
            self.get_logger().error(f"MQTT connect failed: {rc}")

    def mqtt_on_subscribe(self, client, userdata, mid, granted_qos):
        ready = bool(granted_qos) and all(qos < 128 for qos in granted_qos)
        self.elevator_link.connection(ready)
        self.get_logger().info(f"MQTT subscribed: {ready}; chờ heartbeat ESP v2.")

    def mqtt_on_disconnect(self, client, userdata, rc):
        self.elevator_link.connection(False)
        self.get_logger().warning(f"MQTT disconnected: {rc}")

    def mqtt_on_message(self, client, userdata, msg):
        if msg.topic != STATUS_TOPIC:
            return
        try:
            if self.elevator_link.receive(msg.payload.decode(), retained=msg.retain):
                snapshot = self.elevator_link.snapshot
                self.elevator_floor = snapshot["current_floor"]
                self.elevator_state = ("MOVING" if snapshot["moving"] else
                                       "DOOR_OPEN" if snapshot["door_state"] == "OPEN" else "DOOR_CLOSED")
        except (ValueError, TypeError, KeyError) as error:
            self.get_logger().warning(f"Bỏ bản tin MQTT không hợp lệ: {error}")

    def request_elevator_status(self, check=True, *, max_age=None):
        if max_age is not None:
            if check:
                self._check_running()
            snapshot = self.elevator_link.fresh_snapshot(max_age)
            if snapshot is not None:
                return snapshot
        return self.elevator_link.request(
            "GET_STATUS", trip_id=self.trip_id, timeout=8.0,
            check=self._check_running if check else lambda: None,
        )

    def call_elevator(self, floor, timeout=60.0):
        self.current_step = "WAITING_ELEVATOR"

        def on_accepted():
            self.current_step = "ELEVATOR_MOVING"
            self.get_logger().info(f"Thang đã nhận lệnh di chuyển tới tầng {floor}.")

        return self.elevator_link.request(
            "CALL_ELEVATOR", trip_id=self.trip_id, destination_floor=floor,
            timeout=timeout, check=self._check_running, on_accepted=on_accepted,
        )

    # ========================================================
    # ROOM MANAGEMENT
    # ========================================================

    def get_room_info(self, room):

        room = str(room)

        for floor in [1, 2]:

            floor_key = f"floor_{floor}"

            if floor_key not in self.config:
                continue

            floor_config = self.config[floor_key]

            if "rooms" not in floor_config:
                continue

            if room in floor_config["rooms"]:

                return (
                    floor,
                    floor_config["rooms"][room]
                )

        return None


    # ========================================================
    # NAVIGATION
    # ========================================================

    def navigate_to_pose(
        self,
        floor,
        pose_name,
        room=None
    ):

        self._check_running()
        self.current_step = "GOING_TO_ROOM" if pose_name == "room" else "GOING_TO_WAIT"
        floor_key = f"floor_{floor}"

        if floor_key not in self.config:

            self.get_logger().error(
                f"Không có {floor_key} trong YAML."
            )

            return False

        # ====================================================
        # Lấy pose
        # ====================================================

        if pose_name == "room":

            if room is None:

                self.get_logger().error(
                    "navigate_to_pose(): room chưa được cung cấp."
                )

                return False

            room = str(room)

            if "rooms" not in self.config[floor_key]:

                self.get_logger().error(
                    f"{floor_key} không có mục rooms."
                )

                return False

            if room not in self.config[floor_key]["rooms"]:

                self.get_logger().error(
                    f"Không có phòng {room} trong {floor_key}."
                )

                return False

            pose = self.config[
                floor_key
            ]["rooms"][room]

            pose_display_name = f"room {room}"

        else:

            if pose_name not in self.config[floor_key]:

                self.get_logger().error(
                    f"Không có pose '{pose_name}' trong {floor_key}."
                )

                return False

            pose = self.config[
                floor_key
            ][pose_name]

            pose_display_name = pose_name

        # ====================================================
        # Đọc tọa độ
        # ====================================================

        x = float(pose["x"])
        y = float(pose["y"])
        yaw = float(pose["yaw"])

        self.get_logger().info(
            f"Navigation -> {pose_display_name}"
        )

        self.get_logger().info(
            f"x={x:.5f}, y={y:.5f}, yaw={yaw:.5f}"
        )

        # ====================================================
        # Tạo Nav2 goal
        # ====================================================

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()

        goal_msg.pose.header.frame_id = "map"

        goal_msg.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0

        goal_msg.pose.pose.orientation.z = (
            math.sin(yaw / 2.0)
        )

        goal_msg.pose.pose.orientation.w = (
            math.cos(yaw / 2.0)
        )

        # ====================================================
        # Send goal
        # ====================================================

        result = self._execute_action(self.nav_client, goal_msg, 300.0,
                                      f"Navigation {pose_display_name}")

        if result is None:

            self.get_logger().error(
                "Không nhận được Nav2 result."
            )

            return False

        status = result.status

        # Nav2 SUCCEEDED = 4

        if status == 4:

            self.get_logger().info(
                f"ROBOT ĐÃ TỚI {pose_display_name}."
            )

            return True

        self.get_logger().error(
            f"Navigation {pose_display_name} thất bại. "
            f"status={status}"
        )

        return False


    # ========================================================
    # RPP PATH FOLLOWING
    # ========================================================

    @staticmethod
    def interpolate_path(waypoints, num_points=RPP_NUM_PATH_POINTS,
                         preserve_final_orientation=True):
        """Nội suy (x, y, qz, qw) theo chiều dài cung bằng CubicSpline."""
        if len(waypoints) < 2:
            raise ValueError("Cần ít nhất 2 waypoint để nội suy.")
        num_points = max(int(num_points), len(waypoints))
        coords = np.asarray(
            [(float(x), float(y)) for x, y, qz, qw in waypoints]
        )
        segment_lengths = np.linalg.norm(np.diff(coords, axis=0), axis=1)
        if np.any(segment_lengths < 1e-6):
            raise ValueError("Có waypoint liên tiếp trùng hoặc quá gần nhau.")

        distances = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        spline_x = CubicSpline(distances, coords[:, 0], bc_type="natural")
        spline_y = CubicSpline(distances, coords[:, 1], bc_type="natural")
        t_fine = np.linspace(0.0, distances[-1], num_points)
        x_fine, y_fine = spline_x(t_fine), spline_y(t_fine)
        yaw = np.arctan2(spline_y(t_fine, 1), spline_x(t_fine, 1))
        qz_fine, qw_fine = np.sin(yaw / 2.0), np.cos(yaw / 2.0)

        if preserve_final_orientation:
            qz_fine[-1] = float(waypoints[-1][2])
            qw_fine[-1] = float(waypoints[-1][3])

        return [
            (float(x), float(y), float(qz), float(qw))
            for x, y, qz, qw in zip(x_fine, y_fine, qz_fine, qw_fine)
        ]

    def follow_rpp_path(self, floor, path_type):
        """Thực thi entry/exit bằng Nav2 FollowPath và controller ElevatorEntry."""
        if floor not in self.rpp_paths or path_type not in ("entry", "exit"):
            self.get_logger().error(
                f"Không có quỹ đạo RPP hợp lệ: floor={floor}, type={path_type}"
            )
            return False

        if not self.follow_path_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("Không tìm thấy action /robot3/follow_path.")
            return False

        try:
            waypoints = self.rpp_paths[floor][path_type]
            dense_path = self.interpolate_path(waypoints)
        except Exception as error:
            self.get_logger().error(f"Lỗi nội suy RPP {path_type}: {error}")
            return False

        path = PathMsg()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, qz, qw in dense_path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = RPP_CONTROLLER_ID

        self.get_logger().info(
            f"Bắt đầu RPP {path_type.upper()} tầng {floor}: "
            f"{len(waypoints)} waypoint -> {len(path.poses)} pose."
        )

        wrapped_result = self._execute_action(
            self.follow_path_client, goal, 120.0, f"RPP {path_type}"
        )

        if wrapped_result is None:
            self.get_logger().error(f"Không nhận được kết quả RPP {path_type}.")
            return False

        result = wrapped_result.result
        self.get_logger().info(
            f"RPP {path_type} status={wrapped_result.status}, "
            f"error_code={result.error_code}, error_msg='{result.error_msg}'"
        )
        if wrapped_result.status != 4:  # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().error(f"Quỹ đạo RPP {path_type} thất bại.")
            return False

        self.get_logger().info(f"Đã hoàn thành quỹ đạo RPP {path_type.upper()}.")
        return True

    def exit_elevator(self, floor):
        self.current_step = "EXITING"
        self.get_logger().info(
            f"ROBOT ĐANG ĐI RA KHỎI THANG BẰNG RPP - FLOOR {floor}..."
        )
        return self.follow_rpp_path(floor, "exit")


    # ========================================================
    # GO TO WAITING POSITION
    # ========================================================

    def go_to_waiting_position(self):
        self.current_step = "GOING_TO_WAIT"

        self.robot_at_waiting_position = False

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            f"ĐI TỚI VỊ TRÍ CHỜ THANG - "
            f"FLOOR {self.robot_floor}"
        )

        self.get_logger().info(
            "=========================================="
        )

        success = (
            self.navigate_to_pose(
                self.robot_floor,
                "approach_pose"
            )
        )

        if success:

            self.robot_at_waiting_position = True

            self.get_logger().info(
                "ROBOT ĐÃ ĐẾN VỊ TRÍ "
                "CHỜ THANG."
            )

        else:

            self.get_logger().error(
                "Robot không tới được "
                "vị trí chờ thang."
            )

        return success


    # ========================================================
    # OPEN DOOR
    # ========================================================

    def open_door(self, floor):
        self.current_step = "OPENING_DOOR"
        self.elevator_link.request(
            "OPEN_DOOR", trip_id=self.trip_id, expected_floor=floor,
            timeout=15.0, check=self._check_running,
        )
        self.elevator_link.expect_hold(floor)
        return True

    def close_door(self, floor):
        self.current_step = "CLOSING_DOOR"
        self._check_running()
        # Closing is intentional; the heartbeat must no longer demand HOLD.
        self.elevator_link.expect_hold(None)
        self.elevator_link.request(
            "CLOSE_DOOR", trip_id=self.trip_id, expected_floor=floor,
            timeout=15.0, check=self._check_running,
        )
        return True

    # ========================================================
    # ENTER ELEVATOR
    # ========================================================

    def enter_elevator(self):
        self.current_step = "ENTERING"

        self.robot_inside_elevator = False
        self.get_logger().info("ROBOT ĐANG ĐI VÀO THANG BẰNG RPP...")

        success = self.follow_rpp_path(self.robot_floor, "entry")
        if success:
            self.robot_inside_elevator = True
            self.get_logger().info(
                "ROBOT ĐÃ VÀO THANG THEO QUỸ ĐẠO RPP."
            )
            self._pause(2.0)
            return True

        self.get_logger().error("RPP không hoàn thành quỹ đạo đi vào thang.")
        return False


    def _go_to_room(self, destination_room):

        destination_room = str(destination_room)

        # ----------------------------------------------------
        # Tìm phòng
        # ----------------------------------------------------

        room_info = self.get_room_info(destination_room)

        if room_info is None:

            self.get_logger().error(
                f"Không tìm thấy phòng {destination_room}."
            )

            return False

        destination_floor, room_pose = room_info

        current_floor = self.robot_floor

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            f"YÊU CẦU ĐẾN PHÒNG {destination_room}"
        )

        self.get_logger().info(
            f"Robot hiện tại : tầng {current_floor}"
        )

        self.get_logger().info(
            f"Phòng đích     : tầng {destination_floor}"
        )

        self.get_logger().info(
            "=========================================="
        )

        # ====================================================
        # TRƯỜNG HỢP 1: Cùng tầng
        # ====================================================

        if destination_floor == current_floor:

            self.get_logger().info(
                f"Phòng {destination_room} cùng tầng với robot."
            )

            self.destination_room = int(destination_room)

            try:

                success = self.navigate_to_pose(
                    current_floor,
                    "room",
                    destination_room
                )

                if not success:

                    raise RuntimeError(
                        f"Không tới được phòng {destination_room}."
                    )

                self.get_logger().info(
                    "=========================================="
                )

                self.get_logger().info(
                    f"ROBOT ĐÃ TỚI PHÒNG {destination_room}"
                )

                self.get_logger().info(
                    "=========================================="
                )

                return True

            except Exception as e:

                self.get_logger().error(
                    f"Đi tới phòng thất bại: {e}"
                )

                raise

            finally:

                self.destination_floor = None
                self.destination_room = None

        # ====================================================
        # TRƯỜNG HỢP 2: Khác tầng
        # ====================================================

        self.get_logger().info(
            f"Phòng {destination_room} ở tầng khác."
        )

        self.get_logger().info(
            "Bắt đầu quy trình thang máy..."
        )

        self.destination_room = int(destination_room)

        success = self._start_trip(destination_floor)

        if not success:

            self.destination_room = None

            self.get_logger().error(
                "Không thể chuyển tầng."
            )

            return False

        # ====================================================
        # Sau khi robot đã RPP đi ra khỏi thang và tới phòng
        # ====================================================

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            f"ĐÃ ĐẾN TẦNG {destination_floor}"
        )

        self.get_logger().info(
            f"TIẾP TỤC ĐI TỚI PHÒNG {destination_room}"
        )

        self.get_logger().info(
            "=========================================="
        )

        # start_trip() chỉ trả về True sau khi robot đã RPP đi ra,
        # đóng cửa thang và Nav2 tới phòng đích thành công.
        self.get_logger().info(
            "=========================================="
        )
        self.get_logger().info(
            f"ROBOT ĐÃ TỚI PHÒNG {destination_room}"
        )
        self.get_logger().info(
            "=========================================="
        )

        self.destination_room = None
        return True


    # ========================================================
    # INITIALIZE AMCL AFTER FLOOR/MAP CHANGE
    # ========================================================

    def initialize_amcl_at_parking_pose(
        self,
        floor,
        timeout=15.0
    ):
        """
        Sau khi nạp bản đồ tầng đích, đặt initial pose AMCL
        đúng tại waypoint parking (entry_waypoints[-1]).
        """

        self._check_running()
        if floor not in self.rpp_paths:
            self.get_logger().error(
                f"Không có waypoint parking cho floor_{floor}."
            )
            return False

        self.current_step = "LOCALIZING"
        # Dùng trực tiếp tọa độ và quaternion [qz, qw] từ waypoint.
        x, y, qz, qw = self.rpp_paths[floor]["parking_pose"]

        self.get_logger().info(
            "Khởi tạo AMCL đúng tại waypoint parking: "
            f"floor={floor}, x={x:.3f}, y={y:.3f}, "
            f"qz={qz:.6f}, qw={qw:.6f}"
        )

        # Đợi map_server/AMCL nhận bản đồ mới trước khi gửi initialpose.
        self._pause(1.0)

        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = "map"
        initial.pose.pose.position.x = x
        initial.pose.pose.position.y = y
        initial.pose.pose.position.z = 0.0
        initial.pose.pose.orientation.x = 0.0
        initial.pose.pose.orientation.y = 0.0
        initial.pose.pose.orientation.z = qz
        initial.pose.pose.orientation.w = qw

        # Covariance: x/y ~ 0.2 m std dev; yaw ~ 10 degrees std dev.
        covariance = [0.0] * 36
        covariance[0] = 0.04
        covariance[7] = 0.04
        covariance[35] = 0.03046
        initial.pose.covariance = covariance

        publish_started_at = time.monotonic()

        # Publish several times because initialpose is not latched.
        for _ in range(5):
            initial.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(initial)
            self._pause(0.3)

        self.get_logger().info(
            "Đã gửi /robot3/initialpose 5 lần; "
            "chờ AMCL cập nhật và kiểm tra pose."
        )

        start = time.monotonic()
        stable_since = None
        last_log = 0.0

        while time.monotonic() - start < timeout:
            now = time.monotonic()

            pose_is_fresh = (
                self.amcl_pose_received_at >= publish_started_at
            )

            if (
                pose_is_fresh
                and self.robot_x is not None
                and self.robot_y is not None
            ):
                distance = math.hypot(
                    self.robot_x - x,
                    self.robot_y - y
                )

                if now - last_log >= 1.0:
                    self.get_logger().info(
                        f"AMCL so với parking_pose: {distance:.3f} m"
                    )
                    last_log = now

                # Do robot vẫn ở trong cabin, pose nên gần parking_pose.
                if distance <= 0.75:
                    if stable_since is None:
                        stable_since = now
                    elif now - stable_since >= 1.0:
                        self.get_logger().info(
                            "AMCL đã cập nhật gần parking_pose "
                            "và giữ ổn định."
                        )
                        self._pause(AMCL_STABILIZE_TIME)
                        return True
                else:
                    stable_since = None

            self._pause(0.2)

        self.get_logger().error(
            "AMCL không hội tụ gần parking_pose của tầng đích. "
            "Dừng trước khi gửi goal Nav2 để tránh robot chạy sai. "
            "Hãy kiểm tra robot còn ở đúng vị trí trong cabin, "
            "map đã nạp đúng và tọa độ parking_pose."
        )
        return False


    # ========================================================
    # WAIT ROBOT EXIT ELEVATOR
    # ========================================================

    def wait_for_robot_exit(
        self,
        floor,
        timeout=120.0,
        rpp_exit_completed=False
    ):

        self._check_running()
        if floor not in self.rpp_paths:
            self.get_logger().error(
                f"Không có waypoint parking cho floor_{floor}."
            )
            return False

        parking_x, parking_y = self.rpp_paths[floor]["parking_pose"][:2]

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "ĐANG CHỜ ROBOT ĐI RA "
            "KHỎI THANG..."
        )

        self.get_logger().info(
            f"Parking pose: "
            f"x={parking_x:.3f}, "
            f"y={parking_y:.3f}"
        )

        self.get_logger().info(
            f"Ngưỡng rời thang: "
            f"{EXIT_DISTANCE_THRESHOLD:.2f} m"
        )

        self.get_logger().info(
            f"Phải duy trì ngoài ngưỡng "
            f"{EXIT_CONFIRM_TIME:.1f} giây."
        )

        self.get_logger().info(
            f"Chờ AMCL ổn định "
            f"{AMCL_STABILIZE_TIME:.1f} giây "
            "sau khi đổi map."
        )

        self.get_logger().info(
            "=========================================="
        )

        # AMCL đã được khởi tạo và kiểm tra sau khi đổi map
        # trong initialize_amcl_at_parking_pose(). Không chờ thêm ở đây,
        # vì robot có thể bắt đầu rời cabin ngay khi Nav2 nhận goal.
        self.get_logger().info(
            "Bắt đầu kiểm tra robot rời khỏi thang."
        )

        start = time.monotonic()

        last_log_time = 0.0

        # Khi RPP exit đã SUCCEEDED, ta biết robot vừa đi từ parking
        # theo đúng exit path; không cần đợi vòng lặp bắt gặp lại trạng thái
        # "gần parking" như cơ chế Nav2 cũ (vốn theo dõi trong lúc robot di chuyển).
        seen_near_parking = bool(rpp_exit_completed)

        outside_start_time = None

        while (
            time.monotonic() - start
            < timeout
        ):

            if (
                self.robot_x is not None
                and self.robot_y is not None
            ):

                distance = math.hypot(
                    self.robot_x - parking_x,
                    self.robot_y - parking_y
                )

                now = time.monotonic()

                if (
                    now - last_log_time
                    >= 1.0
                ):

                    self.get_logger().info(
                        f"Khoảng cách tới "
                        f"parking_pose: "
                        f"{distance:.3f} m"
                    )

                    last_log_time = now

                # ------------------------------------------------
                # Robot phải từng ở gần parking pose
                # ------------------------------------------------

                if (
                    not seen_near_parking
                    and distance
                    < EXIT_DISTANCE_THRESHOLD
                ):

                    seen_near_parking = True
                    outside_start_time = None

                    self.get_logger().info(
                        "Đã xác nhận robot đang ở "
                        "vùng parking_pose."
                    )

                # ------------------------------------------------
                # Kiểm tra robot rời thang
                # ------------------------------------------------

                if seen_near_parking:

                    if (
                        distance
                        >= EXIT_DISTANCE_THRESHOLD
                    ):

                        if outside_start_time is None:

                            outside_start_time = time.monotonic()

                            self.get_logger().info(
                                "Robot đã vượt ngưỡng "
                                f"{EXIT_DISTANCE_THRESHOLD:.2f} m. "
                                "Bắt đầu xác nhận..."
                            )

                        else:

                            outside_duration = (
                                time.monotonic()
                                - outside_start_time
                            )

                            if (
                                outside_duration
                                >= EXIT_CONFIRM_TIME
                            ):

                                self.get_logger().info(
                                    "=========================================="
                                )

                                self.get_logger().info(
                                    "ĐÃ PHÁT HIỆN ROBOT "
                                    "RỜI KHỎI THANG."
                                )

                                self.get_logger().info(
                                    f"Khoảng cách hiện tại: "
                                    f"{distance:.3f} m"
                                )

                                self.get_logger().info(
                                    f"Đã duy trì ngoài ngưỡng "
                                    f"{outside_duration:.1f} giây."
                                )

                                self.get_logger().info(
                                    f"Chờ {EXIT_DOOR_DELAY:.1f} "
                                    "giây trước khi đóng cửa..."
                                )

                                self.get_logger().info(
                                    "=========================================="
                                )

                                self._pause(
                                    EXIT_DOOR_DELAY
                                )

                                return True

                    else:

                        if outside_start_time is not None:

                            self.get_logger().info(
                                "Robot quay lại gần "
                                "parking_pose. "
                                "Hủy xác nhận rời thang."
                            )

                        outside_start_time = None

            self._pause(0.2)

        self.get_logger().warn(
            "Timeout: chưa phát hiện "
            "robot rời khỏi thang. "
            "Không tự động đóng cửa."
        )

        return False


    # ========================================================
    # SWITCH MAP
    # ========================================================

    def switch_map(
        self,
        floor
    ):

        self._check_running()
        self.current_step = "SWITCHING_MAP"
        if floor not in MAP_FILES:

            self.get_logger().error(
                f"Không có map cho floor "
                f"{floor}"
            )

            return False

        map_file = (
            MAP_FILES[floor]
        )

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            f"ĐANG ĐỔI MAP -> FLOOR {floor}"
        )

        self.get_logger().info(
            map_file
        )

        self.get_logger().info(
            "=========================================="
        )

        if not self.map_client.wait_for_service(
            timeout_sec=10.0
        ):

            self.get_logger().error(
                "Map service không khả dụng."
            )

            return False

        request = LoadMap.Request()

        request.map_url = map_file

        future = (
            self.map_client.call_async(
                request
            )
        )

        self._pending_map = future
        response = self._wait_future(future, 20.0, "LoadMap")
        self._pending_map = None

        if response is None:

            self.get_logger().error(
                "Không nhận được response "
                "từ map server."
            )

            return False

        if response.result == 0:

            self.get_logger().info(
                f"ĐỔI MAP THÀNH CÔNG "
                f"-> FLOOR {floor}"
            )

            return True

        self.get_logger().error(
            f"Đổi map thất bại. "
            f"result={response.result}"
        )

        return False


    # ========================================================
    # MAIN ELEVATOR TRIP
    # ========================================================

    def _start_trip(
        self,
        destination_floor
    ):

        if destination_floor not in [1, 2]:

            self.get_logger().error(
                "Destination chỉ có thể "
                "là 1 hoặc 2."
            )

            return

        if (
            destination_floor
            == self.robot_floor
        ):

            self.get_logger().warn(
                f"Robot đang ở tầng "
                f"{self.robot_floor}. "
                f"Không cần đi thang."
            )

            return


        self.destination_floor = (
            destination_floor
        )

        self.robot_at_waiting_position = False
        self.robot_inside_elevator = False


        current_floor = (
            self.robot_floor
        )

        try:

            # =================================================
            # STEP 1
            # Robot đi tới vị trí chờ
            # =================================================

            if not self.go_to_waiting_position():

                raise RuntimeError(
                    "Không tới được "
                    "approach_pose."
                )

            # =================================================
            # STEP 2
            # Kiểm tra vị trí hiện tại của thang
            # =================================================

            status = self.request_elevator_status(max_age=2.0)
            if status["moving"]:
                raise RuntimeError("Thang đang di chuyển; chưa thể bắt đầu chuyến.")
            self.elevator_link.begin_guard()
            elevator_floor = status["current_floor"]

            self.get_logger().info(
                f"Robot đang ở floor "
                f"{current_floor}"
            )

            self.get_logger().info(
                f"Elevator đang ở floor "
                f"{elevator_floor}"
            )

            # =================================================
            # STEP 3
            # Nếu thang khác tầng -> gọi thang
            # =================================================

            if (
                elevator_floor
                != current_floor
            ):

                self.get_logger().info(
                    "Thang đang ở tầng khác."
                )

                self.call_elevator(current_floor, timeout=60.0)

            else:

                self.get_logger().info(
                    "Thang đang cùng tầng "
                    "với robot."
                )

            # =================================================
            # STEP 4
            # Mở cửa
            # =================================================

            if not self.open_door(current_floor):

                raise RuntimeError(
                    "Không mở được cửa thang."
                )

            # =================================================
            # STEP 5
            # Robot đi vào parking pose
            # =================================================

            if not self.enter_elevator():

                raise RuntimeError(
                    "Robot không vào được thang."
                )

            # =================================================
            # STEP 6
            # Đóng cửa
            # =================================================

            if not self.close_door(current_floor):

                raise RuntimeError(
                    "Không đóng được cửa thang."
                )

            # =================================================
            # STEP 7
            # Gọi thang tới tầng đích
            # =================================================

            self.get_logger().info(
                f"Gọi thang tới tầng "
                f"{destination_floor}..."
            )

            self.call_elevator(destination_floor, timeout=120.0)

            # =================================================
            # STEP 9
            # Mở cửa tại tầng đích
            # =================================================

            if not self.open_door(destination_floor):

                raise RuntimeError(
                    "Không mở được cửa "
                    "tại tầng đích."
                )

            # =================================================
            # STEP 10
            # Đổi map
            # =================================================

            if not self.switch_map(
                destination_floor
            ):

                raise RuntimeError(
                    "Đổi map thất bại."
                )

            self.robot_floor = (
                destination_floor
            )

            # Robot vừa được đưa tới tầng đích và vẫn ở gần
            # parking_pose trong cabin. Khởi tạo lại AMCL theo map mới.
            if not self.initialize_amcl_at_parking_pose(
                destination_floor
            ):
                raise RuntimeError(
                    "AMCL không định vị được tại parking_pose "
                    "sau khi chuyển bản đồ."
                )

            self.get_logger().info(
                "=========================================="
            )

            self.get_logger().info(
                f"ROBOT ĐÃ CHUYỂN SANG "
                f"FLOOR {destination_floor}"
            )

            self.get_logger().info(
                "MAP ĐÃ ĐƯỢC ĐỔI."
            )

            self.get_logger().info(
                "CỬA ĐANG GIỮ MỞ."
            )

            self.get_logger().info(
                "=========================================="
            )

            # =================================================
            # STEP 11
            # RPP đi ra thang, kết thúc tại approach_pose tầng đích
            # =================================================

            if not self.exit_elevator(destination_floor):
                raise RuntimeError(
                    "RPP không hoàn thành quỹ đạo đi ra thang."
                )

            self.get_logger().info(
                "RPP EXIT hoàn thành thành công. Robot đã đến điểm cuối "
                "quỹ đạo tại approach_pose trong dung sai goal checker."
            )
            self.get_logger().info(
                f"Đợi {EXIT_DOOR_DELAY:.1f} giây sau khi RPP hoàn thành "
                "trước khi đóng cửa..."
            )
            self._pause(EXIT_DOOR_DELAY)

            # STEP 12: Đóng cửa sau khi RPP EXIT thành công và chờ đủ 2 giây.
            self.get_logger().info(
                "Gửi lệnh CLOSE_DOOR sau khi robot hoàn thành RPP EXIT."
            )
            if not self.close_door(destination_floor):
                raise RuntimeError(
                    "Không đóng được cửa sau khi robot ra."
                )

            self.robot_inside_elevator = False
            self.elevator_link.end_guard()

            # =================================================
            # STEP 13
            # Chờ AMCL/LiDAR ổn định tại approach_pose rồi Nav2 tới phòng
            # =================================================

            if self.destination_room is not None:
                self.get_logger().info(
                    f"Đợi {POST_EXIT_SETTLE_TIME:.1f}s tại approach_pose "
                    "trước khi Nav2 tới phòng..."
                )
                self._pause(POST_EXIT_SETTLE_TIME)

                destination_room = str(self.destination_room)
                if not self.navigate_to_pose(
                    destination_floor,
                    "room",
                    destination_room
                ):
                    raise RuntimeError(
                        f"Nav2 không tới được phòng {destination_room}."
                    )

                self.get_logger().info(
                    f"ROBOT ĐÃ TỚI PHÒNG {destination_room}."
                )

            self.get_logger().info(
                "=========================================="
            )
            self.get_logger().info("ROBOT ĐÃ RA KHỎI THANG.")
            self.get_logger().info("CỬA THANG ĐÃ ĐÓNG.")
            self.get_logger().info("CHUYẾN ĐI HOÀN THÀNH.")
            self.get_logger().info(
                "=========================================="
            )

            return True

        except Exception as e:

            self.get_logger().error(
                f"CHUYẾN ĐI THẤT BẠI: {e}"
            )

            raise

        finally:


            self.destination_floor = None


    # ========================================================
    # MENU
    # ========================================================

    def print_status(self):

        print("\n")

        print(
            "=========================================="
        )

        print(
            "             ELEVATOR STATUS"
        )

        print(
            "=========================================="
        )

        print(
            f"Robot floor       : "
            f"{self.robot_floor}"
        )

        print(
            f"Robot waiting     : "
            f"{self.robot_at_waiting_position}"
        )

        print(
            f"Robot inside      : "
            f"{self.robot_inside_elevator}"
        )

        print(
            f"Robot x           : "
            f"{self.robot_x}"
        )

        print(
            f"Robot y           : "
            f"{self.robot_y}"
        )

        print(
            f"Elevator floor    : "
            f"{self.elevator_floor}"
        )

        print(
            f"Elevator state    : "
            f"{self.elevator_state}"
        )

        print(
            f"Trip active       : "
            f"{self.trip_active}"
        )

        print(
            f"Destination floor : "
            f"{self.destination_floor}"
        )

        print(
            f"Destination room  : "
            f"{self.destination_room}"
        )

        print(
            "=========================================="
        )

        print()


    # --------------------------------------------------------

    def menu(self):

        while rclpy.ok():

            print("\n")

            print(
                "=========================================="
            )

            print(
                "          ELEVATOR ROBOT MENU"
            )

            print(
                "=========================================="
            )

            print(
                "1. Xem trạng thái"
            )

            print(
                "2. Đặt tầng hiện tại của robot"
            )

            print(
                "3. Đi tới phòng"
            )

            print(
                "0. Thoát"
            )

            print(
                "=========================================="
            )

            print("4. Hủy chuyến đang chạy")
            print("5. Xác nhận đã kiểm tra robot/map/cửa để phục hồi")
            print(f"Trạng thái: {self.task_state}; bước: {self.current_step}; lỗi: {self.last_error}")

            choice = input(
                "Chọn: "
            ).strip()

            # ------------------------------------------------
            # STATUS
            # ------------------------------------------------

            if choice == "1":

                self.print_status()

                try:
                    self.request_elevator_status(check=False)
                except Exception as error:
                    self.get_logger().warning(f"Không đọc được ESP: {error}")

            # ------------------------------------------------
            # SET FLOOR
            # ------------------------------------------------

            elif choice == "2":

                try:

                    floor = int(
                        input(
                            "Robot đang ở tầng (1/2): "
                        )
                    )

                    if floor not in [1, 2]:

                        print(
                            "Chỉ hỗ trợ tầng 1 hoặc 2."
                        )

                        continue

                    if self.set_floor_manually(floor):

                        self.robot_floor = (
                            floor
                        )

                        self.get_logger().info(
                            f"Robot floor = "
                            f"{floor}"
                        )

                except ValueError:

                    print(
                        "Floor không hợp lệ."
                    )

            # ------------------------------------------------
            # DESTINATION
            # ------------------------------------------------

            elif choice == "3":

                try:

                    destination_room = int(
                        input(
                            "Đi tới phòng (101/102/201/202): "
                        )
                    )

                    if self.get_room_info(
                        destination_room
                    ) is None:

                        print(
                            "Phòng không hợp lệ."
                        )

                        continue

                    self.submit_room(destination_room)

                except ValueError:

                    print(
                        "Phòng không hợp lệ."
                    )

            # ------------------------------------------------
            # EXIT
            # ------------------------------------------------

            elif choice == "4":
                self.cancel_trip()

            elif choice == "5":
                confirmation = input(
                    "Đã kiểm tra robot ở ngoài cabin, đúng tầng/map và AMCL đúng? "
                    "Nhập CONFIRM để cho phép chuyến mới: "
                ).strip()
                if confirmation == "CONFIRM":
                    self.recover()

            elif choice == "0":

                print(
                    "Thoát elevator manager."
                )

                break

            else:

                print(
                    "Lựa chọn không hợp lệ."
                )


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = ElevatorManager()

    # ROS spin chạy riêng
    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )

    ros_thread.start()

    try:

        node.menu()

    except (KeyboardInterrupt, EOFError):

        pass

    finally:

        node.stop_tasks()
        node.mqtt_client.disconnect()
        node.mqtt_client.loop_stop()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()
