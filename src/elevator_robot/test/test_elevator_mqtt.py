"""Protocol tests use a fake transport; no broker or robot commands are sent."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from elevator_robot.elevator_mqtt import ElevatorMqtt


def status(seq=1, **updates):
    data = dict(protocol_version=2, robot_id='robot3', boot_id='boot1',
                status_seq=seq, uptime_ms=1000, current_floor=1, target_floor=1,
                moving=False, door_state='CLOSED', door_mode='CLOSED', hold_session='',
                state='DOOR_CLOSED', result='STATUS', error='')
    data.update(updates)
    return data


def reply(link, request, **updates):
    data = status(link.snapshot['status_seq'] + 1, result='COMPLETED')
    data.update({key: request[key] for key in
                 ('robot_id', 'session_id', 'trip_id', 'request_id', 'command', 'command_seq')})
    data.update(updates)
    link.receive(json.dumps(data))


@pytest.fixture
def link():
    obj = ElevatorMqtt(Mock(return_value=SimpleNamespace(rc=0)), 'robot3',
                       retry_interval=0.01, max_attempts=3)
    obj.connection(True)
    obj.receive(json.dumps(status()))
    return obj


def test_only_matching_completion_satisfies_request(link):
    def publish(payload):
        request = json.loads(payload)
        reply(link, request, request_id='old-id')
        reply(link, request, session_id='other-session')
        reply(link, request, trip_id='old-trip')
        reply(link, request, command='OPEN_DOOR')
        return SimpleNamespace(rc=0)
    link.publish = publish
    with pytest.raises(TimeoutError):
        link.request('GET_STATUS', timeout=0.08)
    assert not link.pending


def test_retry_reuses_identical_envelope_and_completes(link):
    sent = []
    def publish(payload):
        sent.append(payload)
        if len(sent) == 2:
            reply(link, json.loads(payload))
        return SimpleNamespace(rc=0)
    link.publish = publish
    assert link.request('CLOSE_DOOR', expected_floor=1, trip_id='trip1', timeout=1)['door_state'] == 'CLOSED'
    assert len(sent) == 2 and sent[0] == sent[1]


def test_accepted_is_not_completion_and_retries_bounded(link):
    def publish(payload):
        reply(link, json.loads(payload), result='ACCEPTED', moving=True, target_floor=2)
        return SimpleNamespace(rc=0)
    link.publish = Mock(side_effect=publish)
    with pytest.raises(TimeoutError):
        link.request('CALL_ELEVATOR', destination_floor=2, timeout=0.25)
    assert link.publish.call_count == 3


def test_rejected_request_fails_immediately(link):
    def publish(payload):
        reply(link, json.loads(payload), result='REJECTED', error='MOVING')
        return SimpleNamespace(rc=0)
    link.publish = Mock(side_effect=publish)
    with pytest.raises(RuntimeError, match='MOVING'):
        link.request('OPEN_DOOR', expected_floor=1)
    assert link.publish.call_count == 1


def test_door_auto_is_not_robot_hold(link):
    def publish(payload):
        reply(link, json.loads(payload), door_state='OPEN', door_mode='AUTO')
        return SimpleNamespace(rc=0)
    link.publish = publish
    with pytest.raises(RuntimeError, match='không còn phù hợp'):
        link.request('OPEN_DOOR', expected_floor=1)


def test_hold_requires_expected_floor_and_session(link):
    def publish(payload):
        reply(link, json.loads(payload), door_state='OPEN', door_mode='ROBOT_HOLD',
              hold_session=link.session_id)
        return SimpleNamespace(rc=0)
    link.publish = publish
    link.begin_guard()
    link.request('OPEN_DOOR', expected_floor=1)
    link.expect_hold(1)
    link.receive(json.dumps(status(seq=3)))
    with pytest.raises(RuntimeError, match='giữ cửa'):
        link.assert_guard()


def test_disconnect_and_reconnect_latches_guard_failure(link):
    link.begin_guard()
    link.connection(False)
    link.connection(True)
    link.receive(json.dumps(status(seq=2)))
    with pytest.raises(RuntimeError, match='Mất kết nối'):
        link.assert_guard()


def test_reboot_invalidates_inflight_request(link):
    def publish(payload):
        reply(link, json.loads(payload), boot_id='boot2', status_seq=1)
        return SimpleNamespace(rc=0)
    link.publish = publish
    with pytest.raises(RuntimeError, match='thay đổi'):
        link.request('CLOSE_DOOR', expected_floor=1)
    assert not link.receive(json.dumps(status(seq=99)))


def test_stale_heartbeat_requires_recovery(link):
    link.begin_guard()
    link.received_at = 0
    with pytest.raises(RuntimeError, match='heartbeat'):
        link.assert_guard()


def test_retained_and_duplicate_status_do_not_refresh_freshness(link):
    before = link.received_at
    assert not link.receive(json.dumps(status(seq=2)), retained=True)
    assert not link.receive(json.dumps(status(seq=1)))
    assert link.received_at == before


def test_old_protocol_not_accepted(link):
    assert not link.receive(json.dumps({'state': 'DOOR_OPEN', 'current_floor': 1}))


def test_publish_error_and_cancellation_cleanup_pending(link):
    link.publish = Mock(return_value=SimpleNamespace(rc=4))
    with pytest.raises(RuntimeError, match='publish'):
        link.request('GET_STATUS')
    assert not link.pending
    with pytest.raises(RuntimeError, match='cancelled'):
        link.request('GET_STATUS', check=Mock(side_effect=RuntimeError('cancelled')))
    assert not link.pending
