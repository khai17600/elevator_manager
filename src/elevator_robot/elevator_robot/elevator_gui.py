import sys
import json
import time
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32

from PySide6.QtCore import Qt, QTimer, QRectF
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QFont
from PySide6.QtWidgets import (
    QApplication,
    QWidget,
    QFrame,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
)


# ============================================================
# CẤU HÌNH
# ============================================================

# Danh sách phòng theo tầng (phải khớp với elevator.yaml).
# Thêm phòng mới ở đây là GUI tự có nút mới.
ROOMS = {
    2: [201, 202],
    1: [101, 102],
}

# Nếu quá thời gian này mà không nhận status từ manager
# thì coi như mất kết nối.
STATUS_TIMEOUT = 2.0

# Khóa nút tạm thời sau khi bấm, tránh bấm đúp trước khi
# manager kịp báo trip_active = True.
PENDING_TIME = 3.0

# Bảng màu
C = {
    'bg': '#eef1f4',
    'card': '#ffffff',
    'border': '#d5dce3',
    'text': '#1d2a35',
    'muted': '#6b7a89',
    'steel': '#2a6f9e',      # thang máy / màu chủ đạo
    'robot': '#e8833a',      # robot
    'green': '#2f9e6e',
    'amber': '#d9a21b',
    'red': '#d64545',
    'shaft': '#26323d',
    'floor': '#f3f6f9',
}

# Trạng thái thang máy -> (chữ hiển thị, màu)
ELEVATOR_STATES = {
    'UNKNOWN': ('Chưa rõ', C['muted']),
    'ARRIVED': ('Đã đến tầng', C['steel']),
    'DOOR_OPEN': ('Cửa đang mở', C['green']),
    'DOOR_CLOSED': ('Cửa đã đóng', C['amber']),
}

# Các bước của một chuyến đi khác tầng
TRIP_STEPS = [
    'Tới điểm chờ',
    'Gọi thang',
    'Vào thang',
    'Di chuyển',
    'Ra & tới phòng',
]


def tint(color, alpha=40):
    """Trả về chuỗi rgba() của một màu hex, dùng cho stylesheet."""
    c = QColor(color)
    return f'rgba({c.red()},{c.green()},{c.blue()},{alpha})'


STYLE = f"""
QWidget {{
    background: {C['bg']};
    color: {C['text']};
    font-family: 'Segoe UI', 'Ubuntu', 'Noto Sans', sans-serif;
    font-size: 14px;
}}
QLabel {{ background: transparent; }}

QFrame#card {{
    background: {C['card']};
    border: 1px solid {C['border']};
    border-radius: 12px;
}}
QLabel#appTitle {{ font-size: 22px; font-weight: 700; }}
QLabel#appSub {{ color: {C['muted']}; }}
QLabel#cardTitle {{ color: {C['muted']}; font-weight: 600; }}
QLabel#key {{ color: {C['muted']}; }}
QLabel#val {{ font-weight: 600; }}
QLabel#floorTag {{ color: {C['muted']}; font-weight: 600; }}

QPushButton#room {{
    background: {C['card']};
    border: 1px solid {C['border']};
    border-radius: 10px;
    padding: 10px 6px;
    font-size: 15px;
    font-weight: 600;
}}
QPushButton#room:hover {{
    border-color: {C['steel']};
    background: {tint(C['steel'], 18)};
}}
QPushButton#room:pressed {{
    background: {tint(C['steel'], 45)};
}}
QPushButton#room:disabled {{
    background: {C['floor']};
    color: #9aa7b4;
    border-color: {C['border']};
}}
QPushButton#room[target="true"],
QPushButton#room[target="true"]:disabled {{
    background: {tint(C['robot'], 38)};
    border: 2px solid {C['robot']};
    color: {C['text']};
}}
"""


# ============================================================
# ROS 2 NODE
# ============================================================

class ElevatorGUI(Node):

    def __init__(self):
        super().__init__('elevator_gui')

        # GUI -> Manager: gửi SỐ PHÒNG (101, 102, 201, 202...)
        self.destination_pub = self.create_publisher(
            Int32,
            '/elevator/set_destination',
            10
        )

        # Manager -> GUI: trạng thái JSON, publish mỗi 0.5 s
        self.status_sub = self.create_subscription(
            String,
            '/elevator/manager_status',
            self.status_callback,
            10
        )

        self.latest_status = {}
        self.last_status_time = 0.0

    def status_callback(self, msg):
        try:
            self.latest_status = json.loads(msg.data)
            self.last_status_time = time.monotonic()
        except Exception as e:
            self.get_logger().error(
                f'Lỗi đọc trạng thái GUI: {e}'
            )

    def send_destination(self, room):
        msg = Int32()
        msg.data = int(room)

        self.destination_pub.publish(msg)

        self.get_logger().info(
            f'GUI gửi yêu cầu đi tới phòng {room}'
        )


# ============================================================
# WIDGET VẼ TÒA NHÀ + THANG MÁY + ROBOT
# ============================================================

class BuildingView(QWidget):
    """
    Mặt cắt tòa nhà 2 tầng: thang máy ở giữa, robot di chuyển
    theo trạng thái thực tế, phòng đích được tô nổi bật.
    """

    def __init__(self):
        super().__init__()

        self.setMinimumSize(270, 400)

        self.robot_floor = None
        self.elevator_floor = None
        self.elevator_state = 'UNKNOWN'
        self.robot_inside = False
        self.robot_waiting = False
        self.dest_floor = None
        self.dest_room = None
        self.connected = False

        # Vị trí hiển thị của cabin (1.0 = tầng 1, 2.0 = tầng 2),
        # trượt mượt về tầng thật của thang.
        self.car_pos = 1.0
        # 0 = cửa đóng, 1 = cửa mở hoàn toàn
        self.door_open = 0.0

        self.anim = QTimer(self)
        self.anim.timeout.connect(self._tick)
        self.anim.start(30)

    def set_data(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)
        self.update()

    def _tick(self):
        changed = False

        if self.elevator_floor in (1, 2):
            diff = self.elevator_floor - self.car_pos
            if abs(diff) > 0.005:
                step = max(abs(diff) * 0.12, 0.01)
                self.car_pos += step if diff > 0 else -step
                changed = True
            else:
                self.car_pos = float(self.elevator_floor)

        door_target = 1.0 if self.elevator_state == 'DOOR_OPEN' else 0.0
        door_diff = door_target - self.door_open
        if abs(door_diff) > 0.01:
            self.door_open += door_diff * 0.25
            changed = True

        if changed:
            self.update()

    # --------------------------------------------------------

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()

        # Nền card
        p.setPen(QPen(QColor(C['border']), 1))
        p.setBrush(QColor(C['card']))
        p.drawRoundedRect(QRectF(0.5, 0.5, w - 1, h - 1), 12, 12)

        # Tiêu đề
        p.setPen(QColor(C['muted']))
        title_font = QFont(self.font())
        title_font.setBold(True)
        title_font.setPointSize(10)
        p.setFont(title_font)
        p.drawText(QRectF(16, 10, w - 32, 22),
                   Qt.AlignLeft | Qt.AlignVCenter,
                   'Sơ đồ tòa nhà')

        top = 40.0
        bottom = h - 14.0
        floor_h = (bottom - top) / 2.0

        shaft_x = w * 0.40
        shaft_w = w * 0.24

        def band_top(floor_value):
            # tầng 2 ở trên, tầng 1 ở dưới
            return top + (2 - floor_value) * floor_h

        label_font = QFont(self.font())
        label_font.setBold(True)
        label_font.setPointSize(10)

        room_font = QFont(self.font())
        room_font.setPointSize(9)
        room_font.setBold(True)

        # ---------------- Các tầng ----------------
        for floor in (2, 1):
            y = band_top(floor)
            rect = QRectF(12, y + 2, w - 24, floor_h - 4)

            is_dest = (self.dest_floor == floor)

            p.setBrush(QColor(C['floor']))
            p.setPen(QPen(
                QColor(C['robot']) if is_dest else QColor(C['border']),
                2 if is_dest else 1
            ))
            p.drawRoundedRect(rect, 10, 10)

            # Nhãn tầng
            p.setFont(label_font)
            p.setPen(QColor(C['muted']))
            p.drawText(QRectF(20, y + 8, 60, 20),
                       Qt.AlignLeft | Qt.AlignVCenter,
                       f'Tầng {floor}')

            # Các phòng ở bên phải thang
            rx = shaft_x + shaft_w + 14
            rw = w - 24 - rx
            ry = y + 12

            p.setFont(room_font)
            for room in ROOMS.get(floor, []):
                chip = QRectF(rx, ry, rw, 26)
                is_target = (self.dest_room == room)

                if is_target:
                    p.setBrush(QColor(C['robot']))
                    p.setPen(Qt.NoPen)
                    p.drawRoundedRect(chip, 7, 7)
                    p.setPen(QColor('#ffffff'))
                else:
                    p.setBrush(QColor(C['card']))
                    p.setPen(QPen(QColor(C['border']), 1))
                    p.drawRoundedRect(chip, 7, 7)
                    p.setPen(QColor(C['text']))

                p.drawText(chip, Qt.AlignCenter, str(room))
                ry += 32

        # ---------------- Giếng thang ----------------
        shaft_rect = QRectF(shaft_x, top + 4, shaft_w, (bottom - top) - 8)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(C['shaft']))
        p.drawRoundedRect(shaft_rect, 8, 8)

        # ---------------- Cabin ----------------
        car_h = floor_h * 0.66
        car_w = shaft_w - 12
        car_x = shaft_x + 6
        car_y = band_top(self.car_pos) + floor_h - car_h - 12
        car_rect = QRectF(car_x, car_y, car_w, car_h)

        known = self.connected and self.elevator_floor in (1, 2)

        p.setBrush(QColor(C['steel']) if known else QColor('#6b7a89'))
        p.setPen(Qt.NoPen)
        p.drawRoundedRect(car_rect, 6, 6)

        # Lòng cabin (hiện ra khi cửa mở)
        inner = car_rect.adjusted(5, 5, -5, -5)
        p.setBrush(QColor('#dfe8ef'))
        p.drawRoundedRect(inner, 4, 4)

        # Hai cánh cửa
        half = inner.width() / 2.0
        closed_part = half * (1.0 - self.door_open)
        door_color = QColor('#7f93a5')
        p.setBrush(door_color)
        p.drawRect(QRectF(inner.left(), inner.top(),
                          closed_part, inner.height()))
        p.drawRect(QRectF(inner.right() - closed_part, inner.top(),
                          closed_part, inner.height()))

        # ---------------- Robot ----------------
        if self.robot_floor in (1, 2) or self.robot_inside:
            radius = 14.0

            if self.robot_inside:
                cx = car_rect.center().x()
                cy = car_rect.bottom() - radius - 8
            else:
                floor = self.robot_floor
                if self.robot_waiting:
                    cx = shaft_x - radius - 6
                else:
                    cx = 20 + radius + 22
                cy = band_top(floor) + floor_h - radius - 14

            p.setPen(QPen(QColor('#ffffff'), 2))
            p.setBrush(QColor(C['robot']))
            p.drawEllipse(QRectF(cx - radius, cy - radius,
                                 radius * 2, radius * 2))

            p.setPen(QColor('#ffffff'))
            p.setFont(label_font)
            p.drawText(QRectF(cx - radius, cy - radius,
                              radius * 2, radius * 2),
                       Qt.AlignCenter, 'R')

        p.end()


# ============================================================
# WIDGET PHỤ
# ============================================================

class InfoRow(QWidget):
    """Một dòng: tên bên trái, giá trị bên phải."""

    def __init__(self, key):
        super().__init__()

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 3, 0, 3)

        self.key_label = QLabel(key)
        self.key_label.setObjectName('key')

        self.value_label = QLabel('--')
        self.value_label.setObjectName('val')
        self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        layout.addWidget(self.key_label)
        layout.addStretch()
        layout.addWidget(self.value_label)

    def set(self, text, color=None):
        self.value_label.setText(text)
        self.value_label.setStyleSheet(
            f'color: {color};' if color else ''
        )


class StepBar(QWidget):
    """Thanh tiến trình các bước của chuyến đi khác tầng."""

    def __init__(self, steps):
        super().__init__()

        self.steps = steps
        self.labels = []

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        for name in steps:
            label = QLabel(name)
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumHeight(30)
            layout.addWidget(label, 1)
            self.labels.append(label)

        self.set_step(None)

    def set_step(self, current):
        for i, label in enumerate(self.labels):
            name = self.steps[i]

            if current is not None and i < current:
                label.setText(f'✓ {name}')
                color, alpha, weight = C['green'], 28, 600
            elif current is not None and i == current:
                label.setText(name)
                color, alpha, weight = C['robot'], 45, 700
            else:
                label.setText(name)
                color, alpha, weight = C['muted'], 18, 500

            label.setStyleSheet(
                f'background: {tint(color, alpha)};'
                f'color: {color if current is not None and i <= current else C["muted"]};'
                f'border-radius: 8px;'
                f'font-size: 12px;'
                f'font-weight: {weight};'
                f'padding: 2px 6px;'
            )


def make_card(title):
    frame = QFrame()
    frame.setObjectName('card')

    layout = QVBoxLayout(frame)
    layout.setContentsMargins(16, 12, 16, 12)
    layout.setSpacing(4)

    title_label = QLabel(title)
    title_label.setObjectName('cardTitle')
    layout.addWidget(title_label)

    return frame, layout


def set_pill(label, text, color):
    label.setText(text)
    label.setStyleSheet(
        f'background: {tint(color, 32)};'
        f'color: {color};'
        f'border: 1px solid {color};'
        f'border-radius: 12px;'
        f'padding: 4px 14px;'
        f'font-weight: 600;'
    )


# ============================================================
# CỬA SỔ CHÍNH
# ============================================================

class MainWindow(QWidget):

    def __init__(self, ros_node):
        super().__init__()

        self.ros_node = ros_node
        self.pending_until = 0.0
        self.room_buttons = {}

        self.setWindowTitle('TurtleBot 4 - Elevator Control')
        self.resize(940, 660)
        self.setStyleSheet(STYLE)

        self.setup_ui()

        # Cập nhật GUI mỗi 200 ms
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(200)

        self.update_gui()

    # --------------------------------------------------------
    # UI
    # --------------------------------------------------------

    def setup_ui(self):

        root = QVBoxLayout(self)
        root.setContentsMargins(20, 16, 20, 20)
        root.setSpacing(14)

        # ---------------- Header ----------------
        header = QHBoxLayout()

        title_box = QVBoxLayout()
        title_box.setSpacing(0)

        title = QLabel('TurtleBot 4 đi thang máy')
        title.setObjectName('appTitle')

        subtitle = QLabel('Chọn phòng, robot tự lo phần còn lại')
        subtitle.setObjectName('appSub')

        title_box.addWidget(title)
        title_box.addWidget(subtitle)

        self.connection_pill = QLabel()

        header.addLayout(title_box)
        header.addStretch()
        header.addWidget(self.connection_pill, 0, Qt.AlignVCenter)

        root.addLayout(header)

        # ---------------- Thân ----------------
        body = QHBoxLayout()
        body.setSpacing(16)

        self.building = BuildingView()
        self.building.setFixedWidth(290)
        body.addWidget(self.building)

        right = QVBoxLayout()
        right.setSpacing(12)

        # ----- Chuyến đi -----
        trip_card, trip_layout = make_card('Chuyến đi')

        self.banner_label = QLabel('Đang chờ elevator_manager...')
        self.banner_label.setWordWrap(True)
        self.banner_label.setMinimumHeight(40)

        self.step_bar = StepBar(TRIP_STEPS)

        self.target_row = InfoRow('Điểm đến')

        trip_layout.addWidget(self.banner_label)
        trip_layout.addSpacing(4)
        trip_layout.addWidget(self.step_bar)
        trip_layout.addSpacing(4)
        trip_layout.addWidget(self.target_row)

        right.addWidget(trip_card)

        # ----- Robot + Thang máy -----
        info_row = QHBoxLayout()
        info_row.setSpacing(12)

        robot_card, robot_layout = make_card('Robot')

        self.robot_floor_row = InfoRow('Đang ở')
        self.robot_pos_row = InfoRow('Tọa độ')
        self.robot_waiting_row = InfoRow('Tại điểm chờ thang')
        self.robot_inside_row = InfoRow('Trong thang')

        for row in (
            self.robot_floor_row,
            self.robot_pos_row,
            self.robot_waiting_row,
            self.robot_inside_row,
        ):
            robot_layout.addWidget(row)

        elevator_card, elevator_layout = make_card('Thang máy')

        self.elevator_floor_row = InfoRow('Đang ở')
        self.elevator_state_row = InfoRow('Trạng thái')

        elevator_layout.addWidget(self.elevator_floor_row)
        elevator_layout.addWidget(self.elevator_state_row)
        elevator_layout.addStretch()

        info_row.addWidget(robot_card, 1)
        info_row.addWidget(elevator_card, 1)

        right.addLayout(info_row)

        # ----- Chọn phòng -----
        dest_card, dest_layout = make_card('Chọn phòng cần đến')
        dest_layout.setSpacing(8)

        for floor in sorted(ROOMS.keys(), reverse=True):

            floor_label = QLabel(f'Tầng {floor}')
            floor_label.setObjectName('floorTag')
            dest_layout.addWidget(floor_label)

            row = QHBoxLayout()
            row.setSpacing(10)

            for room in ROOMS[floor]:
                button = QPushButton(f'Phòng {room}')
                button.setObjectName('room')
                button.setMinimumHeight(58)
                button.setCursor(Qt.PointingHandCursor)
                button.clicked.connect(
                    lambda checked=False, r=room: self.send_room(r)
                )

                row.addWidget(button, 1)
                self.room_buttons[room] = button

            dest_layout.addLayout(row)

        right.addWidget(dest_card)
        right.addStretch()

        body.addLayout(right, 1)
        root.addLayout(body, 1)

    # --------------------------------------------------------
    # GỬI LỆNH
    # --------------------------------------------------------

    def is_connected(self):
        age = time.monotonic() - self.ros_node.last_status_time
        return bool(self.ros_node.latest_status) and age < STATUS_TIMEOUT

    def send_room(self, room):

        status = self.ros_node.latest_status

        if not self.is_connected():
            return

        if status.get('trip_active', False):
            return

        # Khóa nút ngay, chờ manager báo trip_active = True
        self.pending_until = time.monotonic() + PENDING_TIME

        self.ros_node.send_destination(room)

        self.update_gui()

    # --------------------------------------------------------
    # SUY RA TIẾN TRÌNH CHUYẾN ĐI TỪ STATUS
    # --------------------------------------------------------

    def describe_trip(self, s):
        """
        Manager không publish "bước hiện tại", nên GUI suy ra từ
        các cờ trong status. Trả về (nội dung, màu, chỉ số bước).
        """

        trip_active = s.get('trip_active', False)
        room = s.get('destination_room')
        dest_floor = s.get('destination_floor')
        robot_floor = s.get('robot_floor')
        waiting = s.get('robot_waiting', False)
        inside = s.get('robot_inside', False)
        elevator_state = s.get('elevator_state')

        if time.monotonic() < self.pending_until and not trip_active:
            return 'Đã gửi lệnh, đang chờ manager nhận...', C['amber'], None

        if not trip_active:
            return 'Sẵn sàng nhận lệnh', C['green'], None

        target = f'phòng {room}' if room is not None else f'tầng {dest_floor}'

        # Cùng tầng: manager không dùng thang, destination_floor = None
        if dest_floor is None:
            return (
                f'Đang đi thẳng tới {target} (cùng tầng)',
                C['steel'], None
            )

        if inside and robot_floor == dest_floor:
            return (
                f'Đã tới tầng {dest_floor}, đang ra khỏi thang '
                f'và đi tới {target}',
                C['steel'], 4
            )

        if inside:
            return (
                f'Robot đang trong thang, di chuyển tới tầng {dest_floor}',
                C['steel'], 3
            )

        if waiting and elevator_state == 'DOOR_OPEN':
            return 'Cửa thang đã mở, robot đang vào thang', C['steel'], 2

        if waiting:
            return 'Robot đang chờ thang tới tầng của mình', C['steel'], 1

        return 'Robot đang đi tới điểm chờ thang', C['steel'], 0

    # --------------------------------------------------------
    # CẬP NHẬT GUI
    # --------------------------------------------------------

    def update_gui(self):

        status = self.ros_node.latest_status
        connected = self.is_connected()

        # ---------------- Kết nối ----------------
        if connected:
            set_pill(self.connection_pill, 'Đã kết nối manager', C['green'])
        elif status:
            set_pill(self.connection_pill, 'Mất kết nối manager', C['red'])
        else:
            set_pill(self.connection_pill, 'Đang chờ manager', C['amber'])

        if not connected:

            self.set_banner(
                'Chưa nhận được trạng thái từ elevator_manager.\n'
                'Kiểm tra node manager đã chạy chưa.',
                C['amber'] if not status else C['red']
            )

            self.step_bar.set_step(None)
            self.building.set_data(connected=False)

            for button in self.room_buttons.values():
                button.setEnabled(False)

            return

        # ---------------- Đọc status ----------------
        robot_floor = status.get('robot_floor')
        robot_x = status.get('robot_x')
        robot_y = status.get('robot_y')
        robot_waiting = status.get('robot_waiting', False)
        robot_inside = status.get('robot_inside', False)

        elevator_floor = status.get('elevator_floor')
        elevator_state = status.get('elevator_state', 'UNKNOWN')

        trip_active = status.get('trip_active', False)
        dest_floor = status.get('destination_floor')
        dest_room = status.get('destination_room')

        if trip_active:
            self.pending_until = 0.0

        # ---------------- Chuyến đi ----------------
        text, color, step = self.describe_trip(status)

        self.set_banner(text, color)
        self.step_bar.set_step(step)

        if dest_room is not None:
            floor_of_room = dest_floor if dest_floor is not None else robot_floor
            self.target_row.set(
                f'Phòng {dest_room} (tầng {floor_of_room})'
            )
        elif dest_floor is not None:
            self.target_row.set(f'Tầng {dest_floor}')
        else:
            self.target_row.set('--', C['muted'])

        # ---------------- Robot ----------------
        self.robot_floor_row.set(
            f'Tầng {robot_floor}' if robot_floor is not None else '--'
        )

        if robot_x is not None and robot_y is not None:
            self.robot_pos_row.set(f'x {robot_x:.2f}   y {robot_y:.2f}')
        else:
            self.robot_pos_row.set('--', C['muted'])

        self.robot_waiting_row.set(
            'Có' if robot_waiting else 'Chưa',
            C['green'] if robot_waiting else C['muted']
        )

        self.robot_inside_row.set(
            'Có' if robot_inside else 'Không',
            C['robot'] if robot_inside else C['muted']
        )

        # ---------------- Thang máy ----------------
        self.elevator_floor_row.set(
            f'Tầng {elevator_floor}' if elevator_floor is not None else '--'
        )

        state_text, state_color = ELEVATOR_STATES.get(
            elevator_state,
            (str(elevator_state), C['steel'])
        )
        self.elevator_state_row.set(state_text, state_color)

        # ---------------- Sơ đồ tòa nhà ----------------
        self.building.set_data(
            connected=True,
            robot_floor=robot_floor,
            elevator_floor=elevator_floor,
            elevator_state=elevator_state,
            robot_inside=robot_inside,
            robot_waiting=robot_waiting,
            dest_floor=dest_floor if trip_active else None,
            dest_room=dest_room if trip_active else None,
        )

        # ---------------- Nút chọn phòng ----------------
        pending = time.monotonic() < self.pending_until
        can_click = (not trip_active) and (not pending)

        for room, button in self.room_buttons.items():

            room_floor = room // 100

            if robot_floor is None:
                hint = ''
            elif room_floor == robot_floor:
                hint = 'Đi thẳng'
            else:
                hint = 'Qua thang máy'

            button.setText(
                f'Phòng {room}\n{hint}' if hint else f'Phòng {room}'
            )

            is_target = trip_active and (dest_room == room)
            self.set_target(button, is_target)

            button.setEnabled(can_click)

    # --------------------------------------------------------
    # TIỆN ÍCH
    # --------------------------------------------------------

    def set_banner(self, text, color):
        self.banner_label.setText(text)
        self.banner_label.setStyleSheet(
            f'background: {tint(color, 30)};'
            f'color: {C["text"]};'
            f'border-left: 5px solid {color};'
            f'border-radius: 6px;'
            f'padding: 8px 12px;'
            f'font-size: 15px;'
            f'font-weight: 600;'
        )

    @staticmethod
    def set_target(button, is_target):
        value = 'true' if is_target else 'false'

        if button.property('target') == value:
            return

        button.setProperty('target', value)
        button.style().unpolish(button)
        button.style().polish(button)


# ============================================================
# MAIN
# ============================================================

def main():

    rclpy.init()

    ros_node = ElevatorGUI()

    # ROS2 spin chạy ở thread riêng để không chặn Qt
    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(ros_node,),
        daemon=True
    )

    ros_thread.start()

    app = QApplication(sys.argv)

    window = MainWindow(ros_node)
    window.show()

    exit_code = app.exec()

    ros_node.destroy_node()

    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()