"""Run Python manager protocol against the actual ESP C JSON parser and state machine."""
import ctypes
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from elevator_robot.elevator_mqtt import ElevatorMqtt


@pytest.fixture
def bridge(tmp_path):
    esp = Path(os.environ.get('ESP_PROJECT_DIR', '/home/khai/Do_An/Laptop_Pi5/Esp32_CH340_2'))
    if not (esp / 'src/mqtt/elevator_sync.c').exists():
        pytest.skip('Set ESP_PROJECT_DIR to the protocol-v2 ESP project')
    stubs = Path(__file__).parent / 'esp_host'
    cjson = esp / 'managed_components/espressif__cjson/cJSON'
    libpath = tmp_path / 'libelevator_test.so'
    subprocess.run(['gcc', '-shared', '-fPIC', '-std=c11', '-Wall', '-Wextra',
                    '-I'+str(stubs), '-I'+str(esp/'src'), '-I'+str(cjson),
                    str(stubs/'bridge.c'), str(esp/'src/mqtt/elevator_sync.c'),
                    str(cjson/'cJSON.c'), '-lm', '-o', str(libpath)], check=True)
    lib = ctypes.CDLL(str(libpath))
    callback_type = ctypes.CFUNCTYPE(None, ctypes.c_char_p)
    lib.host_init.argtypes = [callback_type]
    lib.host_tick.argtypes = [ctypes.c_uint]
    lib.host_input.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]
    return lib, callback_type


def test_complete_round_trip_with_fragmentation_and_lost_completion(bridge):
    lib, callback_type = bridge
    outputs, sent = [], []
    drop_completion = [True]
    link = ElevatorMqtt(lambda payload: None, 'robot3', retry_interval=0.01)
    link.connection(True)

    def output(payload):
        data = json.loads(payload)
        outputs.append(data)
        if data.get('command') == 'CALL_ELEVATOR' and data['result'] == 'COMPLETED' and drop_completion[0]:
            drop_completion[0] = False
            return
        link.receive(payload.decode())

    callback = callback_type(output)
    lib.host_init(callback)

    def publish(payload):
        data = payload.encode()
        sent.append(json.loads(payload))
        midpoint = len(data)//2
        lib.host_input(data[:midpoint], midpoint, 0, len(data), 0)
        lib.host_input(data[midpoint:], len(data)-midpoint, midpoint, len(data), 0)
        if sent[-1]['command'] == 'CALL_ELEVATOR':
            lib.host_tick(3000)
        return SimpleNamespace(rc=0)

    link.publish = publish
    link.request('GET_STATUS')
    link.begin_guard()
    for floor, destination in [(1, 2), (2, 1)]:
        link.request('OPEN_DOOR', expected_floor=floor)
        link.expect_hold(floor)
        lib.host_tick(6000)  # Hold must survive the old 3-second auto-close interval.
        link.assert_guard()
        link.expect_hold(None)
        link.request('CLOSE_DOOR', expected_floor=floor)
        response = link.request('CALL_ELEVATOR', destination_floor=destination, timeout=1)
        assert response['current_floor'] == destination and not response['moving']
    calls = [request for request in sent if request['command'] == 'CALL_ELEVATOR']
    assert calls[0] == calls[1]  # Lost completion caused same-id retry, not another movement.
    assert len(calls) == 3
    assert link.snapshot['current_floor'] == 1


def test_firmware_ignores_retained_command_and_reports_already_closed(bridge):
    lib, callback_type = bridge
    responses = []
    callback = callback_type(lambda data: responses.append(json.loads(data)))
    lib.host_init(callback)
    request = dict(protocol_version=2, robot_id='robot3', session_id='s', trip_id='t',
                   request_id='r', command_seq=1, boot_id='hostboot', expires_at_ms=10000,
                   command='CLOSE_DOOR', expected_floor=1)
    payload = json.dumps(request).encode()
    lib.host_input(payload, len(payload), 0, len(payload), 1)
    assert len(responses) == 1
    lib.host_input(payload, len(payload), 0, len(payload), 0)
    assert responses[-1]['result'] == 'COMPLETED'
    assert responses[-1]['request_id'] == 'r'
    assert responses[-1]['door_state'] == 'CLOSED'


def test_heartbeat_enqueues_online_and_is_skipped_offline(bridge):
    lib, callback_type = bridge
    responses = []
    callback = callback_type(lambda data: responses.append(json.loads(data)))
    lib.host_init(callback)
    assert len(responses) == 1
    lib.host_tick(1000)
    assert len(responses) == 2
    lib.host_connection(0)
    lib.host_tick(1000)
    assert len(responses) == 2
    lib.host_connection(1)
    lib.host_tick(1000)
    assert len(responses) == 3
