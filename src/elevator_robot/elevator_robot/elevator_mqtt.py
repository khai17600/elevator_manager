"""Correlated, bounded MQTT requests for the simulated elevator (protocol v2)."""
import json
import threading
import time
import uuid


class ElevatorMqtt:
    VERSION = 2

    def __init__(self, publish, robot_id, retry_interval=2.0, max_attempts=4,
                 heartbeat_timeout=5.0):
        self.publish = publish
        self.robot_id = robot_id
        self.session_id = uuid.uuid4().hex
        self.retry_interval = retry_interval
        self.max_attempts = max_attempts
        self.heartbeat_timeout = heartbeat_timeout
        self.lock = threading.RLock()
        self.connected = False
        self.generation = 0
        self.sequence = 0
        self.snapshot = None
        self.received_at = 0.0
        self.pending = {}
        self.retired_boots = set()
        self.guard = False
        self.guard_failure = None
        self.expected_hold_floor = None

    def connection(self, connected):
        with self.lock:
            if self.connected != connected:
                self.generation += 1
            if self.guard and not connected:
                self.guard_failure = "Mất kết nối MQTT trong chuyến."
            self.connected = connected
            if not connected:
                self.received_at = 0.0

    def receive(self, payload, retained=False):
        if retained:
            return False
        data = json.loads(payload)
        if not isinstance(data, dict) or data.get('protocol_version') != self.VERSION:
            return False
        if data.get('robot_id') != self.robot_id:
            return False
        if (not isinstance(data.get('boot_id'), str) or not data['boot_id'] or
                type(data.get('uptime_ms')) is not int or data['uptime_ms'] < 0 or
                type(data.get('status_seq')) is not int or data['status_seq'] <= 0 or
                type(data.get('current_floor')) is not int or data['current_floor'] not in (1, 2) or
                type(data.get('target_floor')) is not int or data['target_floor'] not in (1, 2) or
                type(data.get('moving')) is not bool or
                data.get('door_state') not in ('OPEN', 'CLOSED') or
                data.get('door_mode') not in ('AUTO', 'ROBOT_HOLD', 'CLOSED') or
                not isinstance(data.get('hold_session'), str) or
                data.get('result') not in ('STATUS', 'ACCEPTED', 'COMPLETED', 'REJECTED')):
            return False
        if ((data['door_state'] == 'CLOSED') != (data['door_mode'] == 'CLOSED') or
                (data['moving'] and data['door_state'] != 'CLOSED')):
            return False
        with self.lock:
            old = self.snapshot
            if data['boot_id'] in self.retired_boots:
                return False
            if old and old['boot_id'] != data['boot_id']:
                self.retired_boots.add(old['boot_id'])
                self.generation += 1
                if self.guard:
                    self.guard_failure = "ESP khởi động lại trong chuyến."
            if old and old['boot_id'] == data['boot_id'] and data['status_seq'] <= old['status_seq']:
                return False
            self.snapshot = data
            self.received_at = time.monotonic()
            if self.guard and self.expected_hold_floor is not None and not self._held(data, self.expected_hold_floor):
                self.guard_failure = "ESP không còn giữ cửa tại tầng robot đang vào/ra."
            pending = self.pending.get(data.get('request_id'))
            if pending:
                request = pending['request']
                keys = ('robot_id', 'session_id', 'trip_id', 'request_id', 'command', 'command_seq')
                if all(data.get(key) == request[key] for key in keys):
                    # Do not replace a terminal response with a delayed ACCEPTED.
                    previous = pending['response']
                    if not previous or previous['result'] == 'ACCEPTED':
                        pending['response'] = data
            return True

    def _held(self, data, floor):
        return (not data['moving'] and data['current_floor'] == floor and
                data['door_mode'] == 'ROBOT_HOLD' and
                data['hold_session'] == self.session_id)

    def healthy(self):
        with self.lock:
            return (self.connected and self.snapshot is not None and
                    time.monotonic() - self.received_at <= self.heartbeat_timeout)

    def begin_guard(self):
        with self.lock:
            if not self.healthy():
                raise RuntimeError("Chưa có trạng thái ESP mới qua MQTT.")
            self.guard_failure = None
            self.expected_hold_floor = None
            self.guard = True

    def end_guard(self):
        with self.lock:
            self.guard = False
            self.expected_hold_floor = None

    def expect_hold(self, floor):
        with self.lock:
            self.expected_hold_floor = floor
            if floor is not None and (not self.snapshot or not self._held(self.snapshot, floor)):
                self.guard_failure = "Chưa xác nhận ESP giữ cửa đúng tầng."
                raise RuntimeError(self.guard_failure)

    def assert_guard(self):
        with self.lock:
            if not self.guard:
                return
            if not self.healthy():
                self.guard_failure = self.guard_failure or "Không nhận heartbeat ESP trong thời gian cho phép."
            if self.guard_failure:
                raise RuntimeError(self.guard_failure)

    def request(self, command, *, trip_id='', destination_floor=None,
                expected_floor=None, timeout=15.0, check=lambda: None):
        check()
        with self.lock:
            if not self.connected:
                raise RuntimeError("MQTT chưa kết nối/subscribed.")
            if command != 'GET_STATUS' and not self.healthy():
                raise RuntimeError("Không gửi lệnh khi trạng thái ESP đã cũ.")
            self.sequence += 1
            request = {
                'protocol_version': self.VERSION, 'robot_id': self.robot_id,
                'session_id': self.session_id, 'trip_id': trip_id,
                'request_id': uuid.uuid4().hex, 'command_seq': self.sequence,
                'boot_id': self.snapshot['boot_id'] if self.snapshot else '',
                'command': command,
            }
            if command != 'GET_STATUS':
                request['expires_at_ms'] = int(
                    self.snapshot['uptime_ms'] +
                    (time.monotonic() - self.received_at + min(timeout, 10.0)) * 1000
                )
            if destination_floor is not None:
                request['destination_floor'] = destination_floor
            if expected_floor is not None:
                request['expected_floor'] = expected_floor
            generation = self.generation
            pending = {'request': request, 'response': None}
            self.pending[request['request_id']] = pending
        payload = json.dumps(request)
        deadline = time.monotonic() + timeout
        attempts, next_send = 0, 0.0
        try:
            while True:
                check()
                with self.lock:
                    if not self.connected or self.generation != generation:
                        raise RuntimeError("Phiên MQTT/ESP thay đổi trong lúc chờ lệnh; cần phục hồi.")
                    response = pending['response']
                    if response and response['result'] == 'REJECTED':
                        raise RuntimeError(f"ESP từ chối {command}: {response.get('error', 'UNKNOWN')}")
                    if response and response['result'] == 'COMPLETED':
                        # Validate both correlated completion and latest actual state.
                        current = self.snapshot
                        if command != 'GET_STATUS' and response['boot_id'] != request['boot_id']:
                            raise RuntimeError("ESP đã đổi phiên khởi động.")
                        if command == 'CALL_ELEVATOR':
                            valid = not current['moving'] and current['current_floor'] == destination_floor
                        elif command == 'OPEN_DOOR':
                            valid = self._held(current, expected_floor)
                        elif command == 'CLOSE_DOOR':
                            valid = (not current['moving'] and current['current_floor'] == expected_floor
                                     and current['door_state'] == 'CLOSED')
                        else:
                            valid = True
                        if not valid:
                            raise RuntimeError(f"Trạng thái ESP không còn phù hợp với {command}.")
                        return dict(current)
                now = time.monotonic()
                if now >= deadline:
                    raise TimeoutError(f"Không nhận COMPLETED cho {command} trong {timeout}s.")
                if now >= next_send and attempts < self.max_attempts:
                    info = self.publish(payload)
                    # Paho rc == 0 only means accepted for delivery, not executed by ESP.
                    if info.rc != 0:
                        raise RuntimeError(f"MQTT publish thất bại: rc={info.rc}")
                    attempts += 1
                    next_send = now + self.retry_interval
                time.sleep(min(0.05, max(0.0, deadline - now)))
        finally:
            with self.lock:
                self.pending.pop(request['request_id'], None)
