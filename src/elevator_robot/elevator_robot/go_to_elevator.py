#!/usr/bin/env python3

import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class GoToElevator(Node):

    def __init__(self):
        super().__init__('go_to_elevator')

        # Nhận floor từ ROS parameter
        self.declare_parameter('floor', 1)
        floor = self.get_parameter('floor').value

        try:
            floor = int(floor)
        except ValueError:
            self.get_logger().error(
                f'Floor không hợp lệ: {floor}'
            )
            rclpy.shutdown()
            return

        if floor not in [1, 2]:
            self.get_logger().error(
                f'Floor phải là 1 hoặc 2, nhưng nhận được: {floor}'
            )
            rclpy.shutdown()
            return

        self.floor = floor

        # File cấu hình
        config_file = '/home/khai/elevator_robot/config/elevator.yaml'

        # Đọc YAML
        try:
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(
                f'Không thể đọc file YAML: {e}'
            )
            rclpy.shutdown()
            return

        # Lấy approach_pose theo floor
        floor_key = f'floor_{self.floor}'
        pose = config[floor_key]['approach_pose']

        self.x = float(pose['x'])
        self.y = float(pose['y'])
        self.yaw = float(pose['yaw'])

        self.get_logger().info(
            f'Floor: {self.floor}'
        )

        self.get_logger().info(
            f'Approach pose: x={self.x:.5f}, '
            f'y={self.y:.5f}, yaw={self.yaw:.5f}'
        )

        # Nav2 action
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            '/robot3/navigate_to_pose'
        )

        self.get_logger().info(
            'Đang chờ Nav2 action /robot3/navigate_to_pose...'
        )

        self.action_client.wait_for_server()

        self.get_logger().info(
            'Đã kết nối Nav2!'
        )

        self.send_goal()

    def send_goal(self):

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Vị trí
        goal_msg.pose.pose.position.x = self.x
        goal_msg.pose.pose.position.y = self.y
        goal_msg.pose.pose.position.z = 0.0

        # Chuyển yaw -> quaternion
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        self.get_logger().info(
            f'Đang gửi goal tới vị trí chờ thang máy floor {self.floor}...'
        )

        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Nav2 từ chối goal!')
            rclpy.shutdown()
            return

        self.get_logger().info(
            'Goal đã được Nav2 chấp nhận.'
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.result_callback
        )

    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback

        distance = feedback.distance_remaining

        self.get_logger().info(
            f'Còn khoảng {distance:.2f} m',
            throttle_duration_sec=2.0
        )

    def result_callback(self, future):

        result = future.result().result
        status = future.result().status

        if status == 4:
            self.get_logger().info(
                '======================================'
            )
            self.get_logger().info(
                f'ROBOT ĐÃ ĐẾN VỊ TRÍ CHỜ THANG MÁY FLOOR {self.floor}!'
            )
            self.get_logger().info(
                '======================================'
            )
        else:
            self.get_logger().warn(
                f'Navigation kết thúc với status = {status}'
            )

        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = GoToElevator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
