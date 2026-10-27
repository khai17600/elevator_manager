#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose, FollowPath


class ElevatorEntryTest(Node):

    def __init__(self):
        super().__init__('elevator_entry_test')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('wait_seconds', 2.0)
        self.declare_parameter('controller_id', 'FollowPath')

        self.frame_id = self.get_parameter('frame_id').value
        self.wait_seconds = float(
            self.get_parameter('wait_seconds').value
        )
        self.controller_id = self.get_parameter(
            'controller_id'
        ).value

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            'follow_path'
        )

        # Điểm chờ thang trên map4
        self.wait_pose = (
            -3.6430519141,
            2.4831066774,
            0.6432314215,
            0.7656718216
        )

        # Quỹ đạo vào thang trên map4
        self.waypoints = [
            (
                -3.6143739390,
                2.4313518325,
                0.6536345957,
                0.7568102902
            ),
            (
                -3.3630724279,
                3.5207691275,
                0.3468062008,
                0.9379368098
            ),
            (
                -3.1952463953,
                3.7045478020,
                0.6257264779,
                0.7800425468
            ),
            (
                -3.1710832605,
                4.2266927880,
                -0.7688762396,
                0.6393976292
            )
        ]

    def create_pose(self, x, y, qz, qw):

        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        return pose

    def navigate_to_waiting_pose(self):

        self.get_logger().info(
            'Đang chờ action NavigateToPose...'
        )

        if not self.navigate_client.wait_for_server(
            timeout_sec=20.0
        ):
            self.get_logger().error(
                'Không tìm thấy action NavigateToPose!'
            )
            return False

        goal = NavigateToPose.Goal()

        goal.pose = self.create_pose(*self.wait_pose)

        future = self.navigate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                'Nav2 từ chối mục tiêu điểm chờ.'
            )
            return False

        self.get_logger().info(
            'Đã gửi mục tiêu điểm chờ. Đang di chuyển...'
        )

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result()

        if (
            result is None
            or result.status != GoalStatus.STATUS_SUCCEEDED
        ):
            self.get_logger().error(
                'Không đến được điểm chờ. Dừng chuỗi di chuyển.'
            )
            return False

        self.get_logger().info(
            'Đã đến điểm chờ thang.'
        )

        return True

    def follow_elevator_path(self):

        self.get_logger().info(
            'Đang chờ action FollowPath...'
        )

        if not self.follow_path_client.wait_for_server(
            timeout_sec=20.0
        ):
            self.get_logger().error(
                'Không tìm thấy action FollowPath!'
            )
            return False

        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for waypoint in self.waypoints:

            pose = self.create_pose(*waypoint)
            pose.header.stamp = path.header.stamp

            path.poses.append(pose)

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.controller_id

        self.get_logger().info(
            'Bắt đầu chạy quỹ đạo vào thang qua 4 waypoint.'
        )

        future = self.follow_path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                'Nav2 từ chối quỹ đạo FollowPath.'
            )
            return False

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result()

        if (
            result is None
            or result.status != GoalStatus.STATUS_SUCCEEDED
        ):
            self.get_logger().error(
                'Quỹ đạo chưa hoàn thành.'
            )
            return False

        self.get_logger().info(
            'Đã hoàn thành quỹ đạo vào thang!'
        )

        return True

    def run_test(self):

        self.get_logger().info(
            '===== BAT DAU TEST ELEVATOR ENTRY ====='
        )

        # Bước 1: Đi đến điểm chờ
        if not self.navigate_to_waiting_pose():
            return False

        # Bước 2: Chờ 2 giây
        self.get_logger().info(
            f'Chờ {self.wait_seconds} giây...'
        )

        time.sleep(self.wait_seconds)

        # Bước 3: Đi theo quỹ đạo waypoint
        if not self.follow_elevator_path():
            return False

        self.get_logger().info(
            '===== TEST HOAN TAT ====='
        )

        return True


def main(args=None):

    rclpy.init(args=args)

    node = ElevatorEntryTest()

    try:
        success = node.run_test()
        return_code = 0 if success else 1

    except KeyboardInterrupt:
        node.get_logger().warning(
            'Đã nhận Ctrl+C.'
        )
        return_code = 130

    except Exception as error:
        node.get_logger().error(
            f'Lỗi: {error}'
        )
        return_code = 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return return_code


if __name__ == '__main__':
    sys.exit(main())