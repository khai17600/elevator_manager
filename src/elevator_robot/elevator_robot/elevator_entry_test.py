#!/usr/bin/env python3
import sys
import time
import yaml
import math

import numpy as np
from scipy.interpolate import CubicSpline

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from nav2_msgs.action import NavigateToPose, FollowPath


def interpolate_path(waypoints, num_points=50, preserve_final_orientation=True):
    """Nội suy (x, y, qz, qw) theo chiều dài cung bằng CubicSpline."""
    if len(waypoints) < 2:
        raise ValueError("Cần ít nhất 2 waypoint để nội suy.")
    if num_points < len(waypoints):
        raise ValueError("num_points phải >= số waypoint gốc.")

    coords = np.asarray([(float(x), float(y)) for x, y, qz, qw in waypoints])
    segment_lengths = np.linalg.norm(np.diff(coords, axis=0), axis=1)

    if np.any(segment_lengths < 1e-6):
        raise ValueError("Có waypoint liên tiếp trùng hoặc quá gần nhau.")

    distances = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    cs_x = CubicSpline(distances, coords[:, 0], bc_type="natural")
    cs_y = CubicSpline(distances, coords[:, 1], bc_type="natural")

    t_fine = np.linspace(0.0, distances[-1], num_points)
    x_fine = cs_x(t_fine)
    y_fine = cs_y(t_fine)

    dx = cs_x(t_fine, 1)
    dy = cs_y(t_fine, 1)
    yaw = np.arctan2(dy, dx)

    qz_fine = np.sin(yaw / 2.0)
    qw_fine = np.cos(yaw / 2.0)

    # Giữ orientation khai báo ở waypoint cuối như thuật toán ban đầu.
    # Khi đi ra, orientation cuối được lấy theo tiếp tuyến của đường đi ngược.
    if preserve_final_orientation:
        qz_fine[-1] = float(waypoints[-1][2])
        qw_fine[-1] = float(waypoints[-1][3])

    return [
        (float(x), float(y), float(qz), float(qw))
        for x, y, qz, qw in zip(x_fine, y_fine, qz_fine, qw_fine)
    ]


class ElevatorEntryTest(Node):
    def __init__(self):
        super().__init__("elevator_entry_test")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("wait_seconds", 2.0)
        self.declare_parameter("controller_id", "ElevatorEntry")
        self.declare_parameter("num_path_points", 50)
        self.declare_parameter("exit_wait_seconds", 3.0)
        self.declare_parameter("approach_staging_distance", 0.8)
        self.declare_parameter("map_id", "map5")
        self.declare_parameter("waypoints_file", "config/elevator_waypoints.yaml")
        self.declare_parameter("elevator_config_file", "config/elevator.yaml")

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.wait_seconds = float(self.get_parameter("wait_seconds").value)
        self.controller_id = str(self.get_parameter("controller_id").value)
        self.num_path_points = int(self.get_parameter("num_path_points").value)
        self.exit_wait_seconds = float(
            self.get_parameter("exit_wait_seconds").value
        )
        self.staging_distance = float(
            self.get_parameter("approach_staging_distance").value
        )
        self.map_id = str(self.get_parameter("map_id").value).strip()
        self.waypoints_file = str(self.get_parameter("waypoints_file").value)
        self.elevator_config_file = str(
            self.get_parameter("elevator_config_file").value
        )

        if self.wait_seconds < 0.0:
            raise ValueError("wait_seconds không được âm.")
        if self.exit_wait_seconds < 0.0:
            raise ValueError("exit_wait_seconds không được âm.")
        if self.num_path_points < 2:
            raise ValueError("num_path_points phải >= 2.")
        if self.staging_distance <= 0.0:
            raise ValueError("approach_staging_distance phải > 0.")

        # Đọc waypoint theo map_id được truyền khi chạy node.
        try:
            with open(self.waypoints_file, "r", encoding="utf-8") as file:
                waypoint_config = yaml.safe_load(file)
            elevator_config = waypoint_config["elevator"]
            if "approach_staging_distance" in elevator_config:
                self.staging_distance = float(
                    elevator_config["approach_staging_distance"]
                )
            map_config = elevator_config["maps"][self.map_id]
            self.waypoints = map_config["entry_waypoints"]
            if not isinstance(self.waypoints, list) or len(self.waypoints) < 2:
                raise ValueError("entry_waypoints phải có ít nhất 2 waypoint.")
            self.waypoints = [tuple(float(value) for value in point)
                              for point in self.waypoints]
            if any(len(point) != 4 for point in self.waypoints):
                raise ValueError("Mỗi waypoint phải có dạng [x, y, qz, qw].")
            self.exit_waypoints = map_config["exit_waypoints"]
            if not isinstance(self.exit_waypoints, list) or len(self.exit_waypoints) < 2:
                raise ValueError("exit_waypoints phải có ít nhất 2 waypoint.")
            self.exit_waypoints = [tuple(float(value) for value in point)
                                   for point in self.exit_waypoints]
            if any(len(point) != 4 for point in self.exit_waypoints):
                raise ValueError("Mỗi exit waypoint phải có dạng [x, y, qz, qw].")
        except Exception as error:
            raise RuntimeError(
                f"Không đọc được waypoint cho {self.map_id} từ "
                f"{self.waypoints_file}: {error}"
            ) from error

        # Lấy approach_pose theo bản đồ đang chọn.
        # Quy ước hiện tại: map4 tương ứng floor_1, map5 tương ứng floor_2.
        map_to_floor = {
            "map4": "floor_1",
            "map5": "floor_2",
        }

        if self.map_id not in map_to_floor:
            raise ValueError(
                f"Chưa khai báo tầng tương ứng với {self.map_id}. "
                f"Các map hiện hỗ trợ: {list(map_to_floor.keys())}"
            )

        floor_key = map_to_floor[self.map_id]

        try:
            with open(self.elevator_config_file, "r", encoding="utf-8") as file:
                floor_config = yaml.safe_load(file)

            approach_pose = floor_config[floor_key]["approach_pose"]
            approach_x = float(approach_pose["x"])
            approach_y = float(approach_pose["y"])
            approach_yaw = float(approach_pose["yaw"])

            # Chuyển yaw trong elevator.yaml sang quaternion 2D.
            approach_qz = float(np.sin(approach_yaw / 2.0))
            approach_qw = float(np.cos(approach_yaw / 2.0))

            self.wait_pose = (
                approach_x, approach_y, approach_qz, approach_qw
            )

            # Nav2 đi đến staging_pose ở phía ngoài cửa; RPP căn chỉnh
            # đoạn cuối từ staging_pose đến approach_pose.
            first_x, first_y = self.waypoints[0][0], self.waypoints[0][1]
            dx = first_x - approach_x
            dy = first_y - approach_y
            length = math.hypot(dx, dy)
            if length < 1e-6:
                raise ValueError("approach_pose trùng waypoint đầu đường vào.")
            ux, uy = dx / length, dy / length
            staging_x = approach_x - ux * self.staging_distance
            staging_y = approach_y - uy * self.staging_distance
            staging_yaw = math.atan2(dy, dx)
            self.staging_pose = (
                staging_x, staging_y,
                float(np.sin(staging_yaw / 2.0)),
                float(np.cos(staging_yaw / 2.0)),
            )
            self.approach_waypoints = [self.staging_pose, self.wait_pose]

            # Thêm approach_pose làm điểm mở đầu của entry_waypoints
            # để quỹ đạo vào thang nối tiếp mượt mà từ chính điểm robot đang đỗ.
            self.waypoints = [self.wait_pose] + self.waypoints
        except Exception as error:
            raise RuntimeError(
                f"Không đọc được approach_pose của {floor_key} "
                f"từ {self.elevator_config_file}: {error}"
            ) from error

        # Với __ns:=/robot3, hai action này là /robot3/navigate_to_pose
        # và /robot3/follow_path.
        self.navigate_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )
        self.follow_path_client = ActionClient(
            self, FollowPath, "follow_path"
        )

        self.get_logger().info(f"Namespace: {self.get_namespace()}")
        self.get_logger().info(
            f"Đã chọn {self.map_id} ({floor_key}); "
            f"nạp {len(self.waypoints)} waypoint từ {self.waypoints_file}"
        )
        self.get_logger().info(
            f"Điểm chờ được nạp từ {self.elevator_config_file}: "
            f"x={self.wait_pose[0]:.4f}, y={self.wait_pose[1]:.4f}"
        )
        self.get_logger().info(
            f"frame_id={self.frame_id}, controller_id={self.controller_id}, "
            f"num_path_points={self.num_path_points}"
        )

    def create_pose(self, x, y, qz, qw, stamp=None):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    @staticmethod
    def status_name(status):
        names = {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED"
        }
        return names.get(status, f"STATUS_{status}")

    def navigate_to_waiting_pose(self):
        self.get_logger().info("Đang chờ action NavigateToPose...")
        if not self.navigate_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("Không tìm thấy action NavigateToPose.")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self.create_pose(*self.wait_pose)

        send_future = self.navigate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Nav2 từ chối mục tiêu điểm chờ.")
            return False

        self.get_logger().info("Đã gửi mục tiêu điểm chờ. Đang di chuyển...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error("Không nhận được kết quả NavigateToPose.")
            return False

        self.get_logger().info(
            f"NavigateToPose status: {self.status_name(wrapped_result.status)}"
        )
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                "Không đến được điểm chờ. Dừng chuỗi di chuyển."
            )
            return False

        self.get_logger().info("Đã đến approach_pose (điểm chờ thang).")
        return True

    def follow_approach_path(self):
        """RPP căn chỉnh đoạn cuối từ staging_pose đến approach_pose."""
        self.get_logger().info("Đang căn chỉnh đến điểm chờ bằng RPP...")
        if not self.follow_path_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("Không tìm thấy action FollowPath.")
            return False

        try:
            dense_waypoints = interpolate_path(
                self.approach_waypoints,
                num_points=max(self.num_path_points, len(self.approach_waypoints)),
                preserve_final_orientation=True
            )
        except Exception as error:
            self.get_logger().error(f"Lỗi nội suy quỹ đạo đến điểm chờ: {error}")
            return False

        path = PathMsg()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for waypoint in dense_waypoints:
            path.poses.append(
                self.create_pose(*waypoint, stamp=path.header.stamp)
            )

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id

        send_future = self.follow_path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Nav2 từ chối quỹ đạo RPP đến điểm chờ.")
            return False

        self.get_logger().info("RPP đã nhận quỹ đạo đến approach_pose.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error("Không nhận được kết quả RPP đến điểm chờ.")
            return False

        self.get_logger().info(
            f"FollowPath approach status: {self.status_name(wrapped_result.status)}"
        )
        self.get_logger().info(
            f"FollowPath approach error_code: {wrapped_result.result.error_code}"
        )
        self.get_logger().info(
            f"FollowPath approach error_msg: '{wrapped_result.result.error_msg}'"
        )
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error("RPP chưa hoàn thành căn chỉnh điểm chờ.")
            return False

        self.get_logger().info("Đã căn chỉnh và đỗ tại approach_pose.")
        return True

    def follow_elevator_path(self):
        self.get_logger().info("Đang chờ action FollowPath...")
        if not self.follow_path_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("Không tìm thấy action FollowPath.")
            return False

        try:
            dense_waypoints = interpolate_path(
                self.waypoints, num_points=self.num_path_points
            )
        except Exception as error:
            self.get_logger().error(f"Lỗi nội suy Path: {error}")
            return False

        path = PathMsg()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for waypoint in dense_waypoints:
            path.poses.append(
                self.create_pose(*waypoint, stamp=path.header.stamp)
            )

        self.get_logger().info(
            f"Path gốc: {len(self.waypoints)} waypoint; "
            f"sau nội suy: {len(path.poses)} pose."
        )
        self.get_logger().info(
            f"Gửi FollowPath với controller_id='{self.controller_id}'."
        )

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id

        send_future = self.follow_path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Nav2 từ chối quỹ đạo FollowPath.")
            return False

        self.get_logger().info(
            "Nav2 đã chấp nhận FollowPath; đang theo dõi kết quả..."
        )
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error("Không nhận được kết quả từ FollowPath.")
            return False

        self.get_logger().info(
            f"FollowPath status: {self.status_name(wrapped_result.status)}"
        )
        self.get_logger().info(
            f"FollowPath error_code: {wrapped_result.result.error_code}"
        )
        self.get_logger().info(
            f"FollowPath error_msg: '{wrapped_result.result.error_msg}'"
        )

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error("Quỹ đạo chưa hoàn thành.")
            return False

        self.get_logger().info("Đã hoàn thành quỹ đạo vào thang!")
        return True

    def follow_elevator_exit_path(self):
        """Đi từ waypoint đỗ trong thang, qua đường ra riêng đến approach_pose."""
        self.get_logger().info("Đang chuẩn bị quỹ đạo đi ra khỏi thang...")

        if not self.follow_path_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("Không tìm thấy action FollowPath.")
            return False

        # exit_waypoints được khai báo riêng trong YAML:
        # waypoint4 (điểm đỗ) -> waypoint3 -> waypoint2 -> waypoint1
        # -> approach_pose (điểm chờ thang).
        try:
            dense_waypoints = interpolate_path(
                self.exit_waypoints,
                num_points=max(self.num_path_points, len(self.exit_waypoints)),
                preserve_final_orientation=True
            )
        except Exception as error:
            self.get_logger().error(f"Lỗi nội suy Path đi ra: {error}")
            return False

        path = PathMsg()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for waypoint in dense_waypoints:
            path.poses.append(
                self.create_pose(*waypoint, stamp=path.header.stamp)
            )

        self.get_logger().info(
            f"Path đi ra: {len(self.exit_waypoints)} waypoint riêng; "
            f"sau nội suy: {len(path.poses)} pose."
        )
        self.get_logger().info(
            f"Gửi FollowPath đi ra với controller_id='{self.controller_id}'."
        )

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id

        send_future = self.follow_path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Nav2 từ chối quỹ đạo FollowPath đi ra.")
            return False

        self.get_logger().info(
            "Nav2 đã chấp nhận quỹ đạo đi ra; đang theo dõi kết quả..."
        )
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error(
                "Không nhận được kết quả FollowPath đi ra."
            )
            return False

        self.get_logger().info(
            f"FollowPath exit status: {self.status_name(wrapped_result.status)}"
        )
        self.get_logger().info(
            f"FollowPath exit error_code: {wrapped_result.result.error_code}"
        )
        self.get_logger().info(
            f"FollowPath exit error_msg: '{wrapped_result.result.error_msg}'"
        )

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error("Quỹ đạo đi ra chưa hoàn thành.")
            return False

        self.get_logger().info("Đã hoàn thành quỹ đạo đi ra khỏi thang!")
        return True

    def run_test(self):
        self.get_logger().info("===== BAT DAU TEST ELEVATOR ENTRY =====")

        if not self.navigate_to_waiting_pose():
            return False

        self.get_logger().info(f"Chờ {self.wait_seconds} giây...")
        time.sleep(self.wait_seconds)

        if not self.follow_elevator_path():
            return False

        self.get_logger().info(
            f"Đã vào thang. Chờ {self.exit_wait_seconds} giây trước khi đi ra..."
        )
        time.sleep(self.exit_wait_seconds)

        if not self.follow_elevator_exit_path():
            return False

        self.get_logger().info("===== TEST RA VAO THANG HOAN TAT =====")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = None
    return_code = 1

    try:
        node = ElevatorEntryTest()
        return_code = 0 if node.run_test() else 1
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning(
                "Đã nhận Ctrl+C. Hãy xác nhận robot đã dừng an toàn."
            )
        return_code = 130
    except Exception as error:
        if node is not None:
            node.get_logger().error(f"Lỗi: {error}")
        else:
            print(f"Lỗi khởi tạo node: {error}", file=sys.stderr)
        return_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return return_code


if __name__ == "__main__":
    sys.exit(main())
