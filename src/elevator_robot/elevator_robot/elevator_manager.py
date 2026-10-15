#!/usr/bin/env python3

import json
import math
import threading
import time
import yaml

import paho.mqtt.client as mqtt

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap

# ============================================================
# GUI ROS2 MESSAGE
# ============================================================

from std_msgs.msg import String, Int32


# ============================================================
# CONFIG
# ============================================================

MQTT_BROKER = "10.11.55.162"
MQTT_PORT = 1883

REQUEST_TOPIC = "elevator/request"
STATUS_TOPIC = "elevator/status"

ROBOT_ID = "robot3"

CONFIG_FILE = "/home/khai/elevator_robot/config/elevator.yaml"

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

        self.last_arrived_floor = None

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

        self.mqtt_client.on_connect = (
            self.mqtt_on_connect
        )

        self.mqtt_client.on_message = (
            self.mqtt_on_message
        )

        try:

            self.mqtt_client.connect(
                MQTT_BROKER,
                MQTT_PORT,
                60
            )

            self.mqtt_client.loop_start()

            self.get_logger().info(
                f"Đã kết nối MQTT broker "
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

        threading.Thread(
            target=self.go_to_room,
            args=(destination_room,),
            daemon=True
        ).start()


    # ========================================================
    # MANAGER -> GUI
    # ========================================================

    def publish_gui_status(self):

        data = {
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

    def mqtt_on_connect(
        self,
        client,
        userdata,
        flags,
        rc
    ):

        if rc == 0:

            self.get_logger().info(
                "MQTT connected."
            )

            client.subscribe(
                STATUS_TOPIC
            )

            self.get_logger().info(
                f"Subscribed: {STATUS_TOPIC}"
            )

            self.request_elevator_status()

        else:

            self.get_logger().error(
                f"MQTT connect failed, rc={rc}"
            )


    # --------------------------------------------------------

    def mqtt_on_message(
        self,
        client,
        userdata,
        msg
    ):

        try:

            data = json.loads(
                msg.payload.decode()
            )

            self.get_logger().info(
                f"MQTT STATUS: {data}"
            )

            # -----------------------------------------------
            # Elevator floor
            # -----------------------------------------------

            if "current_floor" in data:

                self.elevator_floor = int(
                    data["current_floor"]
                )

            # -----------------------------------------------
            # Elevator state
            # -----------------------------------------------

            if "state" in data:

                self.elevator_state = (
                    data["state"]
                )

            # -----------------------------------------------
            # ARRIVED
            # -----------------------------------------------

            if self.elevator_state == "ARRIVED":

                if self.elevator_floor is not None:

                    self.last_arrived_floor = (
                        self.elevator_floor
                    )

                    self.get_logger().info(
                        f"THANG ĐÃ ĐẾN TẦNG "
                        f"{self.elevator_floor}"
                    )

        except Exception as e:

            self.get_logger().error(
                f"Lỗi xử lý MQTT: {e}"
            )


    # ========================================================
    # MQTT COMMAND
    # ========================================================

    def publish_command(
        self,
        command,
        destination_floor=None
    ):

        data = {
            "robot_id": ROBOT_ID,
            "command": command
        }

        if destination_floor is not None:

            data["destination_floor"] = int(
                destination_floor
            )

        payload = json.dumps(data)

        self.get_logger().info(
            f"MQTT COMMAND -> {payload}"
        )

        self.mqtt_client.publish(
            REQUEST_TOPIC,
            payload,
            qos=1
        )


    # --------------------------------------------------------

    def request_elevator_status(self):

        self.publish_command(
            "GET_STATUS"
        )


    # ========================================================
    # WAIT ELEVATOR
    # ========================================================

    def wait_for_elevator_arrival(
        self,
        target_floor,
        timeout=60
    ):

        self.get_logger().info(
            f"Đang chờ thang tới tầng "
            f"{target_floor}..."
        )

        start = time.time()

        while time.time() - start < timeout:

            if (
                self.last_arrived_floor
                == target_floor
            ):

                self.get_logger().info(
                    f"Thang đã tới tầng "
                    f"{target_floor}."
                )

                return True

            time.sleep(0.2)

        self.get_logger().error(
            f"Timeout: thang không tới "
            f"tầng {target_floor}"
        )

        return False


    # ========================================================
    # WAIT ELEVATOR STATE
    # ========================================================

    def wait_for_elevator_state(
        self,
        target_state,
        timeout=15
    ):

        self.get_logger().info(
            f"Đang chờ elevator state = "
            f"{target_state}"
        )

        start = time.time()

        while time.time() - start < timeout:

            if self.elevator_state == target_state:

                self.get_logger().info(
                    f"Elevator state = "
                    f"{target_state}"
                )

                return True

            time.sleep(0.2)

        self.get_logger().warn(
            f"Timeout chờ state "
            f"{target_state}"
        )

        return False


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

        send_future = (
            self.nav_client.send_goal_async(goal_msg)
        )

        while not send_future.done():

            time.sleep(0.1)

        goal_handle = send_future.result()

        if goal_handle is None:

            self.get_logger().error(
                "Không nhận được goal handle."
            )

            return False

        if not goal_handle.accepted:

            self.get_logger().error(
                f"Nav2 từ chối goal {pose_display_name}."
            )

            return False

        self.get_logger().info(
            f"Nav2 đã nhận goal {pose_display_name}."
        )

        # ====================================================
        # Wait result
        # ====================================================

        result_future = goal_handle.get_result_async()

        while not result_future.done():

            time.sleep(0.2)

        result = result_future.result()

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
    # GO TO WAITING POSITION
    # ========================================================

    def go_to_waiting_position(self):

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

    def open_door(self):

        self.get_logger().info(
            "Gửi lệnh OPEN_DOOR..."
        )

        self.publish_command(
            "OPEN_DOOR"
        )

        return (
            self.wait_for_elevator_state(
                "DOOR_OPEN",
                timeout=15
            )
        )


    # ========================================================
    # CLOSE DOOR
    # ========================================================

    def close_door(self):

        self.get_logger().info(
            "Gửi lệnh CLOSE_DOOR..."
        )

        self.publish_command(
            "CLOSE_DOOR"
        )

        return (
            self.wait_for_elevator_state(
                "DOOR_CLOSED",
                timeout=15
            )
        )


    # ========================================================
    # ENTER ELEVATOR
    # ========================================================

    def enter_elevator(self):

        self.robot_inside_elevator = False

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "ROBOT ĐANG ĐI VÀO THANG MÁY..."
        )

        self.get_logger().info(
            "=========================================="
        )

        success = (
            self.navigate_to_pose(
                self.robot_floor,
                "parking_pose"
            )
        )

        if success:

            self.robot_inside_elevator = True

            self.get_logger().info(
                "ROBOT ĐÃ VÀO ĐÚNG "
                "PARKING POSE."
            )

            time.sleep(2.0)

            return True

        self.get_logger().error(
            "Robot không tới được "
            "parking_pose."
        )

        return False


    # ========================================================
    # GO TO ROOM
    # ========================================================

    def go_to_room(self, destination_room):

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

            self.trip_active = True
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

                return False

            finally:

                self.trip_active = False
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

        success = self.start_trip(destination_floor)

        if not success:

            self.destination_room = None

            self.get_logger().error(
                "Không thể chuyển tầng."
            )

            return False

        # ====================================================
        # Sau khi robot đã ra khỏi thang
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

        # start_trip() đã điều hướng tới phòng đích trên thread riêng
        # và chỉ trả về True sau khi Nav2 tới phòng thành công.
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
        Sau khi nạp bản đồ tầng đích, đặt initial pose của AMCL
        tại parking_pose của tầng đó. Chỉ an toàn khi robot thực
        sự vẫn ở gần vị trí parking trong cabin thang máy.
        """

        floor_key = f"floor_{floor}"

        if floor_key not in self.config:
            self.get_logger().error(
                f"Không có {floor_key} trong YAML."
            )
            return False

        parking = self.config[floor_key].get("parking_pose")

        if not parking:
            self.get_logger().error(
                f"{floor_key} không có parking_pose."
            )
            return False

        x = float(parking["x"])
        y = float(parking["y"])
        yaw = float(parking["yaw"])

        self.get_logger().info(
            "Khởi tạo AMCL tại parking_pose của bản đồ mới: "
            f"floor={floor}, x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )

        # Đợi map_server/AMCL nhận bản đồ mới trước khi gửi initialpose.
        time.sleep(1.0)

        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = "map"
        initial.pose.pose.position.x = x
        initial.pose.pose.position.y = y
        initial.pose.pose.position.z = 0.0
        initial.pose.pose.orientation.x = 0.0
        initial.pose.pose.orientation.y = 0.0
        initial.pose.pose.orientation.z = math.sin(yaw / 2.0)
        initial.pose.pose.orientation.w = math.cos(yaw / 2.0)

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
            time.sleep(0.3)

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
                        time.sleep(AMCL_STABILIZE_TIME)
                        return True
                else:
                    stable_since = None

            time.sleep(0.2)

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
        timeout=120.0
    ):

        floor_key = (
            f"floor_{floor}"
        )

        if floor_key not in self.config:

            self.get_logger().error(
                f"Không có {floor_key} "
                f"trong YAML."
            )

            return False

        parking = self.config[
            floor_key
        ]["parking_pose"]

        parking_x = float(
            parking["x"]
        )

        parking_y = float(
            parking["y"]
        )

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

        start = time.time()

        last_log_time = 0.0

        seen_near_parking = False

        outside_start_time = None

        while (
            time.time() - start
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

                now = time.time()

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

                            outside_start_time = time.time()

                            self.get_logger().info(
                                "Robot đã vượt ngưỡng "
                                f"{EXIT_DISTANCE_THRESHOLD:.2f} m. "
                                "Bắt đầu xác nhận..."
                            )

                        else:

                            outside_duration = (
                                time.time()
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

                                time.sleep(
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

            time.sleep(0.2)

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

        while not future.done():

            time.sleep(0.1)

        response = (
            future.result()
        )

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

    def start_trip(
        self,
        destination_floor
    ):

        if self.trip_active:

            self.get_logger().warn(
                "Đang có một chuyến đi."
            )

            return

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

        self.trip_active = True

        self.destination_floor = (
            destination_floor
        )

        self.robot_at_waiting_position = False
        self.robot_inside_elevator = False

        self.last_arrived_floor = None

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

            self.request_elevator_status()

            time.sleep(1.0)

            elevator_floor = (
                self.elevator_floor
            )

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

                self.last_arrived_floor = (
                    None
                )

                self.publish_command(
                    "CALL_ELEVATOR",
                    current_floor
                )

                if not self.wait_for_elevator_arrival(
                    current_floor,
                    timeout=60
                ):

                    raise RuntimeError(
                        "Không gọi được thang "
                        "tới tầng robot."
                    )

            else:

                self.get_logger().info(
                    "Thang đang cùng tầng "
                    "với robot."
                )

            # =================================================
            # STEP 4
            # Mở cửa
            # =================================================

            if not self.open_door():

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

            if not self.close_door():

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

            self.last_arrived_floor = (
                None
            )

            self.publish_command(
                "CALL_ELEVATOR",
                destination_floor
            )

            # =================================================
            # STEP 8
            # Chờ ARRIVED
            # =================================================

            if not self.wait_for_elevator_arrival(
                destination_floor,
                timeout=120
            ):

                raise RuntimeError(
                    f"Thang không tới tầng "
                    f"{destination_floor}."
                )

            # =================================================
            # STEP 9
            # Mở cửa tại tầng đích
            # =================================================

            if not self.open_door():

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
            # Nếu có phòng đích -> tự động Nav2 tới phòng
            # =================================================

            if self.destination_room is not None:

                destination_room = str(
                    self.destination_room
                )

                self.get_logger().info(
                    "=========================================="
                )

                self.get_logger().info(
                    f"TỰ ĐỘNG ĐI TỚI PHÒNG "
                    f"{destination_room}"
                )

                self.get_logger().info(
                    "Robot sẽ tự Nav2 từ "
                    "parking_pose tới phòng đích."
                )

                self.get_logger().info(
                    "=========================================="
                )

                # Bắt đầu Nav2 tới phòng trên thread riêng để trong lúc
                # robot đang chạy ra khỏi cabin, manager vẫn theo dõi
                # khoảng cách tới parking_pose và đóng cửa đúng lúc.
                self._destination_nav_result = {"success": None}
                self._destination_nav_done = threading.Event()

                def navigate_to_destination_room():
                    try:
                        self._destination_nav_result["success"] = (
                            self.navigate_to_pose(
                                destination_floor,
                                "room",
                                destination_room
                            )
                        )
                    except Exception as nav_error:
                        self.get_logger().error(
                            f"Lỗi điều hướng tới phòng: {nav_error}"
                        )
                        self._destination_nav_result["success"] = False
                    finally:
                        self._destination_nav_done.set()

                self._destination_nav_thread = threading.Thread(
                    target=navigate_to_destination_room,
                    daemon=True
                )
                self._destination_nav_thread.start()

                self.get_logger().info(
                    "Nav2 đang điều hướng tới phòng trong khi manager "
                    "theo dõi robot rời cabin."
                )

            else:

                # -------------------------------------------------
                # Không có phòng đích:
                # giữ nguyên chế độ cũ - chờ robot tự đi ra.
                # -------------------------------------------------

                if not self.wait_for_robot_exit(
                    destination_floor,
                    timeout=120.0
                ):

                    raise RuntimeError(
                        "Không phát hiện robot "
                        "rời khỏi thang."
                    )

            # =================================================
            # STEP 12
            # Theo dõi robot ra khỏi cabin rồi đóng cửa.
            # Nếu đang Nav2 tới phòng, thread Nav2 vẫn tiếp tục chạy.
            # =================================================

            if self.destination_room is not None:
                # Nếu Nav2 đã kết thúc trước khi phát hiện robot ra ngoài,
                # vẫn kiểm tra khoảng cách AMCL; không tự đóng cửa khi
                # chưa xác nhận robot đã rời vùng parking.
                exit_detected = self.wait_for_robot_exit(
                    destination_floor,
                    timeout=120.0
                )

                if not exit_detected:
                    raise RuntimeError(
                        "Không xác nhận được robot rời khỏi thang."
                    )
            else:
                # Chuyến chuyển tầng không gắn với phòng đích.
                exit_detected = self.wait_for_robot_exit(
                    destination_floor,
                    timeout=120.0
                )

                if not exit_detected:
                    raise RuntimeError(
                        "Không phát hiện robot rời khỏi thang."
                    )

            self.get_logger().info(
                "Robot đã rời khỏi vùng thang. Gửi lệnh đóng cửa..."
            )

            if not self.close_door():
                raise RuntimeError(
                    "Không đóng được cửa sau khi robot ra."
                )

            self.robot_inside_elevator = False

            # Đợi Nav2 tới phòng hoàn tất; việc đóng cửa đã diễn ra
            # ngay sau khi robot rời cabin, không chờ tới tận phòng.
            if self.destination_room is not None:
                nav_done = getattr(
                    self,
                    "_destination_nav_done",
                    None
                )
                nav_thread = getattr(
                    self,
                    "_destination_nav_thread",
                    None
                )
                nav_result = getattr(
                    self,
                    "_destination_nav_result",
                    {"success": False}
                )

                if nav_done is not None:
                    if not nav_done.wait(timeout=180.0):
                        raise RuntimeError(
                            "Nav2 vẫn chưa hoàn tất sau 180 giây."
                        )

                if nav_thread is not None:
                    nav_thread.join(timeout=0.1)

                if not nav_result.get("success", False):
                    raise RuntimeError(
                        f"Nav2 không tới được phòng {self.destination_room}."
                    )

                self.get_logger().info(
                    f"ROBOT ĐÃ TỚI PHÒNG {self.destination_room}."
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

            return False

        finally:

            self.trip_active = False

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

            choice = input(
                "Chọn: "
            ).strip()

            # ------------------------------------------------
            # STATUS
            # ------------------------------------------------

            if choice == "1":

                self.print_status()

                self.request_elevator_status()

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

                    if self.switch_map(
                        floor
                    ):

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

                    threading.Thread(
                        target=self.go_to_room,
                        args=(destination_room,),
                        daemon=True
                    ).start()

                except ValueError:

                    print(
                        "Phòng không hợp lệ."
                    )

            # ------------------------------------------------
            # EXIT
            # ------------------------------------------------

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

    except KeyboardInterrupt:

        pass

    finally:

        node.mqtt_client.loop_stop()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()