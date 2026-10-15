#!/usr/bin/env python3

import json
import threading
import time

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap

import paho.mqtt.client as mqtt


class MapSwitcher(Node):

    def __init__(self):
        super().__init__('map_switcher')

        # ==========================================================
        # ROS
        # ==========================================================

        self.map_service_name = '/robot3/map_server/load_map'

        self.get_logger().info(
            f'Đang chờ {self.map_service_name}...'
        )

        self.load_map_client = self.create_client(
            LoadMap,
            self.map_service_name
        )

        while not self.load_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'Đang chờ {self.map_service_name}...'
            )

        self.get_logger().info('Đã kết nối Map Server!')

        # ==========================================================
        # MAP
        # ==========================================================

        self.maps = {
            1: '/home/khai/new_map.yaml',
            2: '/home/khai/new_map2.yaml',
        }

        self.current_floor = 1

        # ==========================================================
        # MQTT
        # ==========================================================

        self.mqtt_broker = '10.11.62.44'
        self.mqtt_port = 1883

        self.status_topic = 'elevator/status'
        self.request_topic = 'elevator/request'

        self.get_logger().info(
            f'Đang kết nối MQTT '
            f'{self.mqtt_broker}:{self.mqtt_port}...'
        )

        # Paho version đang có trên máy bạn là API cũ
        self.mqtt_client = mqtt.Client(
            client_id='robot3_map_switcher'
        )

        self.mqtt_client.on_connect = self.mqtt_on_connect
        self.mqtt_client.on_message = self.mqtt_on_message
        self.mqtt_client.on_disconnect = self.mqtt_on_disconnect

        try:
            self.mqtt_client.connect(
                self.mqtt_broker,
                self.mqtt_port,
                60
            )

            self.mqtt_client.loop_start()

        except Exception as e:
            self.get_logger().error(
                f'Không thể kết nối MQTT: {e}'
            )

        # ==========================================================
        # MENU THREAD
        # ==========================================================

        self.menu_thread = threading.Thread(
            target=self.control_menu,
            daemon=True
        )

        self.menu_thread.start()

    # ==============================================================
    # MQTT CALLBACK
    # ==============================================================

    def mqtt_on_connect(self, client, userdata, flags, rc):

        if rc == 0:

            self.get_logger().info(
                'MQTT kết nối thành công!'
            )

            client.subscribe(self.status_topic)

            self.get_logger().info(
                f'Đã subscribe: {self.status_topic}'
            )

        else:

            self.get_logger().error(
                f'MQTT kết nối thất bại, rc={rc}'
            )

    # ==============================================================

    def mqtt_on_disconnect(self, client, userdata, rc):

        self.get_logger().warn(
            f'MQTT bị ngắt kết nối, rc={rc}'
        )

    # ==============================================================

    def mqtt_on_message(self, client, userdata, msg):

        try:

            payload = msg.payload.decode()

            self.get_logger().info(
                f'MQTT status: {payload}'
            )

            data = json.loads(payload)

            floor = data.get('current_floor')
            state = data.get('state')

            if floor is None or state is None:
                return

            self.get_logger().info(
                f'Thông tin thang: tầng={floor}, state={state}'
            )

            # ======================================================
            # CHỈ ĐỔI MAP KHI THANG ĐÃ ĐẾN TẦNG
            # ======================================================

            if state == 'ARRIVED':

                if floor not in self.maps:

                    self.get_logger().warn(
                        f'Không có map cho tầng {floor}'
                    )

                    return

                if floor == self.current_floor:

                    self.get_logger().info(
                        f'Robot đã ở map tầng {floor}, '
                        f'không cần đổi map.'
                    )

                    return

                self.get_logger().info(
                    f'Thang đã đến tầng {floor}. '
                    f'Đang chuyển map...'
                )

                self.switch_map(floor)

        except Exception as e:

            self.get_logger().error(
                f'Lỗi xử lý MQTT: {e}'
            )

    # ==============================================================
    # SWITCH MAP
    # ==============================================================

    def switch_map(self, floor):

        if floor not in self.maps:

            self.get_logger().error(
                f'Không tìm thấy map tầng {floor}'
            )

            return

        map_file = self.maps[floor]

        self.get_logger().info(
            f'Load map tầng {floor}: {map_file}'
        )

        request = LoadMap.Request()

        request.map_url = map_file

        future = self.load_map_client.call_async(request)

        future.add_done_callback(
            lambda f: self.map_load_callback(f, floor)
        )

    # ==============================================================

    def map_load_callback(self, future, floor):

        try:

            response = future.result()

            if response.result == 0:

                self.current_floor = floor

                self.get_logger().info(
                    '======================================'
                )

                self.get_logger().info(
                    f'ĐÃ ĐỔI SANG MAP TẦNG {floor}'
                )

                self.get_logger().info(
                    '======================================'
                )

                self.get_logger().info(
                    'Hãy dùng 2D Pose Estimate để '
                    'định vị robot trên map mới.'
                )

            else:

                self.get_logger().error(
                    f'Load map thất bại. result={response.result}'
                )

        except Exception as e:

            self.get_logger().error(
                f'Lỗi load map: {e}'
            )

    # ==============================================================
    # GỬI MQTT REQUEST
    # ==============================================================

    def send_mqtt_command(self, command, destination_floor=None):

        data = {
            'command': command
        }

        if destination_floor is not None:

            data['destination_floor'] = destination_floor

        payload = json.dumps(data)

        result = self.mqtt_client.publish(
            self.request_topic,
            payload,
            qos=1
        )

        if result.rc == mqtt.MQTT_ERR_SUCCESS:

            self.get_logger().info(
                f'Đã gửi MQTT: {payload}'
            )

        else:

            self.get_logger().error(
                f'Gửi MQTT thất bại: rc={result.rc}'
            )

    # ==============================================================
    # MENU ĐIỀU KHIỂN
    # ==============================================================

    def control_menu(self):

        # Đợi MQTT kết nối
        time.sleep(2)

        while rclpy.ok():

            try:

                print('\n')
                print('======================================')
                print('       ĐIỀU KHIỂN THANG MÁY')
                print('======================================')
                print('1. Gọi thang / chọn tầng 1')
                print('2. Gọi thang / chọn tầng 2')
                print('3. Mở / giữ cửa')
                print('4. Đóng cửa')
                print('0. Thoát menu')
                print('======================================')

                choice = input(
                    'Nhập lựa chọn: '
                ).strip()

                # --------------------------------------------------
                # TẦNG 1
                # --------------------------------------------------

                if choice == '1':

                    self.get_logger().info(
                        'Yêu cầu thang đến tầng 1'
                    )

                    self.send_mqtt_command(
                        'CALL_ELEVATOR',
                        1
                    )

                # --------------------------------------------------
                # TẦNG 2
                # --------------------------------------------------

                elif choice == '2':

                    self.get_logger().info(
                        'Yêu cầu thang đến tầng 2'
                    )

                    self.send_mqtt_command(
                        'CALL_ELEVATOR',
                        2
                    )

                # --------------------------------------------------
                # MỞ / GIỮ CỬA
                # --------------------------------------------------

                elif choice == '3':

                    self.get_logger().info(
                        'Yêu cầu MỞ / GIỮ cửa'
                    )

                    self.send_mqtt_command(
                        'OPEN_DOOR'
                    )

                # --------------------------------------------------
                # ĐÓNG CỬA
                # --------------------------------------------------

                elif choice == '4':

                    self.get_logger().info(
                        'Yêu cầu ĐÓNG cửa'
                    )

                    self.send_mqtt_command(
                        'CLOSE_DOOR'
                    )

                # --------------------------------------------------
                # THOÁT
                # --------------------------------------------------

                elif choice == '0':

                    self.get_logger().info(
                        'Thoát menu điều khiển.'
                    )

                    break

                else:

                    print(
                        'Lựa chọn không hợp lệ!'
                    )

            except EOFError:

                break

            except Exception as e:

                self.get_logger().error(
                    f'Lỗi menu: {e}'
                )

    # ==============================================================

    def destroy_node(self):

        try:

            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = MapSwitcher()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
