"""Exercise task ownership and cancellation without ROS or robot hardware."""
import ast
from concurrent.futures import Future
from pathlib import Path
import threading
import time
import uuid
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


@pytest.fixture
def manager():
    # Compile production methods, avoiding ROS imports and Node initialization.
    source = Path(__file__).parents[1] / 'elevator_robot' / 'elevator_manager.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    names = {'_check_running', '_pause', '_wait_future', '_cancel_active_goal',
             '_execute_action', 'submit_room', '_run_room', 'cancel_trip',
             'recover', 'set_floor_manually', '_start_trip'}
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    namespace = dict(threading=threading, time=time, uuid=uuid, EXIT_DOOR_DELAY=2.0, POST_EXIT_SETTLE_TIME=1.5, rclpy=SimpleNamespace(ok=lambda: True))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)
    node = namespace['ElevatorManager']()
    node._task_lock = threading.Lock()
    node._cancel_event = threading.Event()
    node._shutdown_event = threading.Event()
    node._active_goal = node._pending_goal = node._pending_map = None
    node.task_state = node.current_step = 'IDLE'
    node.trip_active = False
    node.elevator_link = Mock(assert_guard=Mock())
    node.get_logger = Mock(return_value=Mock())
    node.get_room_info = Mock(return_value=(1, {}))
    return node


def finished(value):
    future = Future()
    future.set_result(value)
    return future


def test_only_one_room_task_and_manual_map_blocked(manager):
    entered, release = threading.Event(), threading.Event()

    def trip(room):
        entered.set()
        release.wait(2)
        return True

    manager._go_to_room = trip
    assert manager.submit_room(101)
    assert entered.wait(1)
    try:
        assert not manager.submit_room(102)
        assert not manager.set_floor_manually(2)
        assert not manager.recover()
    finally:
        release.set()
        manager._worker.join(2)
    assert manager.task_state == 'COMPLETED'
    assert not manager.trip_active


def test_failure_blocks_new_trip_until_recovery(manager):
    manager._go_to_room = Mock(side_effect=RuntimeError('navigation failed'))
    assert manager.submit_room(201)
    manager._worker.join(2)
    assert manager.task_state == 'RECOVERY_REQUIRED'
    assert manager.last_error == 'navigation failed'
    assert not manager.submit_room(101)
    assert manager.recover()
    assert manager.task_state == 'IDLE'


def test_cancel_interrupts_wait_and_requires_recovery(manager):
    entered = threading.Event()

    def trip(room):
        entered.set()
        manager._pause(30)
        return True

    manager._go_to_room = trip
    manager.submit_room(101)
    assert entered.wait(1)
    manager.cancel_trip()
    manager._worker.join(2)
    assert not manager._worker.is_alive()
    assert manager.task_state == 'RECOVERY_REQUIRED'


def test_future_has_deadline(manager):
    with pytest.raises(TimeoutError):
        manager._wait_future(Future(), 0, 'test')


def test_late_goal_is_cancelled_and_blocks_recovery(manager):
    pending = Future()
    client = Mock(send_goal_async=Mock(return_value=pending))
    manager._wait_future = Mock(side_effect=TimeoutError('acceptance timeout'))
    with pytest.raises(TimeoutError):
        manager._execute_action(client, object(), 10, 'test')
    assert not manager.recover()
    goal = Mock(accepted=True)
    pending.set_result(goal)
    goal.cancel_goal_async.assert_called_once()
    assert manager._active_goal is goal
    assert manager._pending_goal is None


def test_action_result_timeout_cancels_goal(manager):
    goal = Mock(accepted=True)
    client = Mock(send_goal_async=Mock(return_value=finished(goal)))
    manager._wait_future = Mock(side_effect=[goal, TimeoutError(), object(), object()])
    with pytest.raises(TimeoutError):
        manager._execute_action(client, object(), 10, 'test')
    goal.cancel_goal_async.assert_called_once()
    assert manager._active_goal is None


def test_unconfirmed_cancel_keeps_goal_and_recovery_lock(manager):
    manager._active_goal = Mock()
    manager._wait_future = Mock(side_effect=TimeoutError())
    assert not manager.recover()
    assert manager._active_goal is not None


def test_pending_map_blocks_recovery_and_manual_map(manager):
    manager._pending_map = Future()
    assert not manager.recover()
    assert not manager.set_floor_manually(2)


@pytest.mark.parametrize('outcome', ['success', 'exit_failed', 'cancel_during_delay'])
def test_exit_completion_controls_door_closing(manager, outcome):
    manager.robot_floor = 1
    manager.destination_room = None
    manager.go_to_waiting_position = Mock(return_value=True)
    manager.request_elevator_status = Mock(return_value={'moving': False, 'current_floor': 1})
    manager.open_door = Mock(return_value=True)
    manager.enter_elevator = Mock(return_value=True)
    manager.call_elevator = Mock(return_value=True)
    manager.switch_map = Mock(return_value=True)
    manager.initialize_amcl_at_parking_pose = Mock(return_value=True)
    manager.wait_for_robot_exit = Mock(side_effect=AssertionError('Legacy distance check must not run'))
    events = []
    manager.exit_elevator = Mock(side_effect=lambda floor: events.append(('exit', floor)) or outcome != 'exit_failed')
    manager.close_door = Mock(side_effect=lambda floor: events.append(('close', floor)) or True)

    def pause(seconds):
        events.append(('pause', seconds))
        if outcome == 'cancel_during_delay':
            raise RuntimeError('cancelled')

    manager._pause = pause
    if outcome == 'success':
        assert manager._start_trip(2)
        assert events == [('close', 1), ('exit', 2), ('pause', 2.0), ('close', 2)]
        assert manager.robot_inside_elevator is False
        manager.elevator_link.end_guard.assert_called_once()
    else:
        with pytest.raises(RuntimeError):
            manager._start_trip(2)
        assert ('close', 2) not in events
        manager.elevator_link.end_guard.assert_not_called()
    manager.wait_for_robot_exit.assert_not_called()
