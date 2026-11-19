# Robot–ESP MQTT protocol v2

Both manager and firmware must be updated together. Legacy requests/responses are
not accepted. Topics and broker are unchanged: `elevator/request`,
`elevator/status`, `10.11.53.59:1883`. ESP is a simulation: acknowledgements describe
its internal state and LED output, not physical door/floor sensors.

## Request and response

Every request contains `protocol_version: 2`, `robot_id: robot3`, `session_id`
(manager process UUID), `trip_id` (room task UUID, empty for manual reads),
`request_id` (command UUID), strictly increasing `command_seq`, `boot_id`.
Mutations also include `expires_at_ms`, computed from ESP uptime plus local elapsed
time and at most 10 seconds. An unseen expired command is rejected rather than
executed after a long MQTT delay. No wall-clock synchronization is required.

- `GET_STATUS`: discovers current boot/state; `boot_id` may be empty.
- `CALL_ELEVATOR`: includes integer `destination_floor` (1/2).
- `OPEN_DOOR` and `CLOSE_DOOR`: include integer `expected_floor` (1/2).

Status messages contain the protocol/robot/boot identifiers, monotonic
`status_seq` within a boot, `uptime_ms`, `current_floor`, `target_floor`, `moving`,
`door_state` (`OPEN`/`CLOSED`), `door_mode` (`AUTO`/`ROBOT_HOLD`/`CLOSED`),
`hold_session`, and the legacy display field `state`.

Command responses echo request/session/trip identifiers, command and command_seq:
- `ACCEPTED`: moving to destination; not completion.
- `COMPLETED`: operation finished (including already closed/already at floor).
- `REJECTED`: no new operation; `error` states why.

Unsolicited `STATUS` messages report real state every second. `GET_STATUS` never
fabricates `ARRIVED`. The robot verifies completion *and* actual floor/door state;
`AUTO` open does not satisfy a request for robot hold.

## Retry and ordering

Manager publishes QoS1, non-retained, at most four identical envelopes separated
by two seconds. Publish success alone is not ESP acknowledgement. Command replies
use QoS1; heartbeat uses QoS0 with `enqueue(..., store=true)` and is skipped
while disconnected. The outbox remains bounded at 16 KiB. Neither uses retain. ESP rejects retained requests
and reassembles bounded MQTT fragments (maximum 1024 bytes).

ESP remembers the last 16 mutation results, protecting an in-progress entry from
eviction. Duplicate IDs replay the outcome with a fresh state snapshot; they do
not perform the operation again. Reusing an ID with altered content is rejected.
After cache eviction, a per-session sequence high-water mark prevents old commands
from executing again. Eight manager sessions are supported per ESP boot; once
full, new mutation sessions get `SESSION_CAPACITY` (no silent eviction). Reads do
not use cache/session slots. A rejected request stays rejected: a later intentional
attempt needs a new request ID, even when the physical condition has changed.

## Concurrency and failure

MQTT only parses/enqueues copied requests. One hardware task owns movement,
buttons, door mode and deduplication. A three-second nonblocking movement timer
allows status queries/rejections while moving. Buttons cannot override robot hold.
An opening command while moving is rejected. Robot hold is owned by the manager
session; another session cannot release it. Broker loss preserves hold.

Manager guards the elevator part of a journey: disconnect, ESP reboot, missing
heartbeat for five seconds, or losing a confirmed hold aborts the task and cancels
robot navigation, leaving `RECOVERY_REQUIRED`. Reconnection does not automatically
resume a failed trip. This does not cancel an ESP movement already accepted; it
finishes in the simulator. A queued unseen command can still run within its short
validity window. Check actual robot/ESP state before manual recovery.

After manager restart while a previous session owns HOLD, check the robot is clear
before resetting the ESP simulation to release that hold. The new session cannot
silently take ownership. Reset during a trip causes recovery, never automatic retry
against the new boot id. Cached outcomes are RAM-only and are isolated by boot ID.

## Build and verification

Robot:

```bash
cd ~/elevator_robot
python3 -m pytest -q src/elevator_robot/test/test_manager_flow.py src/elevator_robot/test/test_elevator_mqtt.py src/elevator_robot/test/test_mqtt_cross_language.py
source /opt/ros/jazzy/setup.bash
colcon build --packages-select elevator_robot --symlink-install
```

ESP host state-machine tests (from ESP project root):

```bash
gcc -std=c11 -Wall -Wextra -Werror -Isrc/mqtt src/mqtt/elevator_sync.c test/test_elevator_sync.c -o /tmp/test_elevator_sync
/tmp/test_elevator_sync
pio run
```

Flash the ESP firmware using the existing PlatformIO workflow, then restart manager.
No firmware upload is performed by the source-code changes. Test one same-floor
command, one complete two-floor trip, duplicate/lost replies, disconnect while
holding, and ESP reset before using the integrated flow on the robot.

Cross-language tests compile the actual ESP JSON parser/serializer and state machine
into a host shared library; only transport/GPIO are stubbed. Set `ESP_PROJECT_DIR`
if the ESP checkout is not at the default local path. They exercise fragmented
payloads, retained-command rejection and a complete round trip with a lost response.

The current PlatformIO build reports a board/sdkconfig flash-size mismatch (4 MB
versus 2 MB). This predates the protocol changes; confirm the board and flash
configuration before uploading. The build itself succeeds and fits its app partition.
