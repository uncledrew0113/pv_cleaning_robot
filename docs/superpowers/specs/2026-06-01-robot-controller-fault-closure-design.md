# RobotController and Fault Closure Refactor Design

Date: 2026-06-01

## Goal

Refactor the current app, service, middleware, and domain business flow into a simpler and safer architecture for the photovoltaic dry-cleaning robot.

The refactor must preserve the production behavior that matters:

- Remote RPC can start one-way cleaning toward the opposite endpoint.
- Remote RPC can start one-way cleaning toward the primary dock.
- Remote RPC can stop an active mission.
- Remote RPC can start a configured full mission.
- Remote RPC can reset a latched fault from `FaultStopped` to `Idle`.
- Scheduler can start configured missions.
- Proximity sensors still provide hard stop behavior at both endpoints.
- Health telemetry and local JSONL rotation remain available.
- Runtime parameters from cloud shared attributes keep the current effective-time semantics.

The main change is architectural: all business state changes must be serialized through one business controller, while hard safety stop remains allowed to bypass that queue.

## Non-Goals

This refactor will not redesign the already accepted HAL, driver, protocol, and device layers.

This refactor will not add LoRaWAN behavior to the main path. The cloud path should remain transport-adaptable, but the production path is still 4G and ThingsBoard.

This refactor will not implement charging behavior. `Charging` remains in the state enum for future use, but low battery only rejects startup.

This refactor will not add physical midpoint stop triggers. The robot still works only between two endpoint proximity sensors.

## Current Problems

The current implementation can run, but business control is spread across several objects and threads:

- `RobotSupervisor` owns business orchestration.
- `RobotFsm` owns part of the state transitions.
- `FaultService` records and partly decides fault policy.
- `FaultHandler` handles P0/P1/P2 and calls emergency stop or dispatches to the supervisor.
- `SafetyMonitor` publishes events through synchronous `EventBus` callbacks.
- `SchedulerService`, ThingsBoard RPC, watchdog, safety monitor, main-loop ticks, and fault handling can all enter business logic from different threads.

The most important risk is not the number of states. The risk is that state changes can be caused by multiple callback threads. This makes the final behavior harder to reason about, even if individual functions use locks.

## Target Architecture

The refactor keeps the architecture small:

```text
SafetyMonitor
  Hard endpoint safety only.

FaultDetector
  Periodic fault detection only.

FaultPolicy
  Fault table and fault-to-action decision only.

RobotController
  Single business event queue.
  Owns RobotState.
  Owns MissionContext.
  Owns business fault state.
  Emits motion, recovery, fault-clear, and telemetry state changes.

ThingsBoardControlPlane
  Cloud adapter only.
  RPC ingress, shared-attribute ingress, telemetry egress.

HealthService
  Health and diagnostics telemetry.
  Local JSONL rotation.
```

The intended deletions or merges are:

- Replace `RobotSupervisor` with `RobotController`.
- Remove the standalone `RobotFsm` class, unless implementation shows a tiny private helper is clearer. State transitions should be easy to read in one controller file.
- Remove `FaultHandler`; P0/P1/P2 dispatch logic moves into `RobotController + FaultPolicy`.
- Replace `FaultService::decide()` with `FaultPolicy`.
- Stop calling `FaultService::report(P0, ...)` from scattered locations.

## Threading Model

All business state mutation must happen on one business thread:

```text
external thread
    -> RobotController::post(event)
    -> RobotController business loop
    -> state transition
    -> output actions
```

This applies to:

- ThingsBoard RPC callbacks.
- Scheduler callbacks.
- Safety monitor settled or unstable events.
- Watchdog timeout callbacks.
- Fault detector findings.
- Recovery completion.
- Self-check completion.
- Motion start failure.

The only allowed bypass is hard safety stop:

```text
hard safety trigger
    -> direct motor stop path
    -> RobotController::post(event)
```

This means:

- Hard stop is not delayed by the business queue.
- Business state is still changed only by the controller thread.

## Hard Safety Path

`SafetyMonitor` keeps the endpoint hard-stop responsibility.

When an endpoint proximity sensor triggers:

```text
GPIO edge / poll fallback
    -> SafetyMonitor::on_limit_trigger()
    -> walk_group->emergency_override(0.0f)
    -> wait for stable endpoint confirmation
    -> post LimitSettled or LimitUnstable to RobotController
```

`SafetyMonitor` must not:

- Decide whether a mission is complete.
- Decide whether the endpoint is expected.
- Call the state machine directly.
- Report P0 directly as a business decision.

`LimitUnstable` and unexpected endpoint cases become fault facts handled by `FaultPolicy`.

## Robot State Model

The state set is fixed:

```cpp
enum class RobotState {
    Idle,
    SelfChecking,
    ExecutingMission,
    SettlingEndpoint,
    Recovering,
    Charging,
    FaultStopped,
};
```

`Charging` is reserved but not entered in the current main flow.

State meanings:

- `Idle`: no active mission, can accept start commands if position, config, and battery allow.
- `SelfChecking`: startup gate before motion begins.
- `ExecutingMission`: actively executing the current segment.
- `SettlingEndpoint`: endpoint has been reached and stabilized; controller decides next segment or mission completion.
- `Recovering`: recovery motion is running after a recoverable runtime issue.
- `Charging`: reserved for future charging integration.
- `FaultStopped`: latched fault state. Only `fault_reset` can clear it.

## Minimal State Transitions

```text
Idle
  StartRequested -> SelfChecking
  FaultDetected(P0) -> FaultStopped

SelfChecking
  SelfCheckOk -> ExecutingMission
  SelfCheckRejected -> Idle
  SelfCheckFatal -> FaultStopped
  FaultDetected(P0) -> FaultStopped

ExecutingMission
  LimitSettled(expected) -> SettlingEndpoint
  FaultDetected(recoverable) -> Recovering
  FaultDetected(P0) -> FaultStopped
  StopRequested -> Idle

SettlingEndpoint
  MissionDone -> Idle
  NextSegment -> ExecutingMission
  FaultDetected(P0) -> FaultStopped

Recovering
  RecoveryOk -> ExecutingMission
  RecoveryFailed -> FaultStopped
  FaultDetected(P0) -> FaultStopped

FaultStopped
  FaultReset -> Idle

Charging
  No active transition in this refactor.
```

`StopRequested` is only valid while a mission is active. It is intended for remote stop and operator stop. It stops motor and brush, clears the active mission context, and enters `Idle`.

`FaultReset` clears active fault state and mission context, then enters `Idle`. It does not run self-check and does not reboot the device.

## Business Events

The controller event set should be small:

```cpp
CommandRequested
ScheduleWindowHit
LimitSettled
LimitUnstable
WatchdogTimeout
FaultDetected
SelfCheckFinished
RecoveryFinished
MotionStartFailed
Tick
```

`Tick` can be used by the controller thread to drive periodic internal checks, but external threads should not directly call safety or recovery tick functions that mutate state.

## Output Actions

The controller should emit only a small set of actions:

```cpp
StartSegment
StopMotion
EmergencyStopMotion
StartRecovery
ClearFault
PublishSnapshot
```

Avoid adding dedicated actions for return-home, pause, resume, charge, or protective stop in this refactor. Those are either not part of the accepted current flow or can be represented by mission segments and existing actions.

## Mission Semantics

Endpoint names are `A` and `B`.

The physical left and right proximity sensors map to endpoint `A` and endpoint `B` through configuration or wiring assumptions, but business logic should not use left/right as mission semantics.

Supported mission commands:

- `CleanTowardOppositeEndpoint`: clean from current position toward the endpoint opposite the primary dock direction. It can be started even if the robot is not currently at the primary dock.
- `CleanTowardPrimaryDock`: clean toward the primary dock endpoint. It can be started even if the robot is not currently at the opposite endpoint.
- `StartConfiguredMission`: start the configured full mission from the primary dock semantics.
- `Stop`: stop the currently active mission and enter `Idle`.
- `FaultReset`: clear latched fault and enter `Idle`.

Dock mode:

- `single_dock`: one primary dock and one return endpoint. A full configured mission is primary dock -> opposite endpoint -> primary dock.
- `dual_dock`: both endpoints are long-term docking-capable. A full configured mission from one dock to the other is complete when the opposite endpoint is reached.

Startup low battery behavior:

- Low battery rejects start while in `Idle`.
- Low battery does not enter `Charging` in this refactor.

## Fault Closure

Fault handling is a closed loop:

```text
fault fact
    -> FaultDetected event
    -> FaultPolicy::decide()
    -> RobotController transition
    -> output action
    -> telemetry snapshot
```

`FaultDetector` detects facts. It does not change state.

`FaultPolicy` decides severity and action. It does not change state.

`RobotController` applies the decision. It is the only owner of state changes.

## P0 Source Closure

P0 business decisions must be centralized. Instead of scattered calls like:

```cpp
fault->report(Level::P0, code, description);
```

producers should post facts:

```cpp
RobotController::post(FaultDetected{source, code, detail});
```

The initial P0 candidates are:

- `ConflictingLimitSides`
- `UnexpectedEndpoint`
- `LimitUnstableAfterHardStop`
- `WatchdogTimeout`
- `WheelSpinFree`
- `MotorDriverFault`
- `CanCommunicationLost`
- `ImuOffline`
- `BmsCriticalAlarm`
- `BrushCriticalFault`
- `MotionStartFailed`
- `RecoveryFailed`
- `TaskContextInconsistent`

P0 action:

```text
EmergencyStopMotion
clear active mission
enter FaultStopped
record active fault
publish snapshot through telemetry
```

For hard safety triggers, the hard stop may happen before the event reaches the controller. This does not conflict with P0 closure because hard stop and business state are separate responsibilities.

## FaultDetector

`FaultDetector` is a non-real-time detector. It should run from the controller business loop or another non-RT producer that only posts events.

It reads existing status sources:

- walk motor group diagnostics
- brush motor status or diagnostics
- BMS data and diagnostics
- IMU latest data and diagnostics
- GPS diagnostics if needed for current production logic
- NavService pose, including spin-free detection
- endpoint limit state
- watchdog timeout events
- motion start results
- recovery results

It should detect at least:

- BMS critical alarm
- BMS low battery at start gate
- IMU offline or stale
- IMU attitude out of allowed range
- motor driver fault
- CAN communication lost
- brush critical fault
- wheel spin-free
- conflicting limit sides
- unexpected endpoint
- motion start failure
- recovery failure
- task context inconsistency

Recoverable attitude deviation should map to recovery instead of immediate P0 when the device health is otherwise acceptable.

## FaultPolicy

`FaultPolicy` is a simple table. It maps fault facts to controller actions.

Initial action set:

```cpp
enum class FaultAction {
    WarnOnly,
    RejectStart,
    StartRecovery,
    EmergencyStopAndLatch,
};
```

Do not keep brush-off return-home in the main policy for this refactor unless a specific production requirement is reconfirmed. The accepted current model does not include `ReturningHome` as a state.

Default behavior:

- Unknown P0-class fault code: `EmergencyStopAndLatch`.
- Unknown P1/P2/P3-class fault code: `WarnOnly`, unless explicitly listed.
- Recovery failure: `EmergencyStopAndLatch`.

## Self-Check

Self-check is the startup gate. It should be explicit and easy to read.

Checks:

- config is valid
- command is legal
- battery is above minimum start threshold
- endpoint state is not inconsistent
- current position is acceptable for the requested command
- motor drivers are available
- brush is available if the segment requires cleaning
- IMU data is available and fresh
- BMS data is available and not critical
- no active latched fault

Failure behavior:

- Non-fatal rejection: return to `Idle` with rejected reason.
- Fatal self-check failure: enter `FaultStopped`.

## Recovery

Recovery remains a service used by the controller.

Recovery may be multi-stage:

```text
stop
sample
micro-move
verify
```

`RecoveryMotion` should own the recovery sequence mechanics. `RobotController` should only start recovery and receive `RecoveryFinished`.

Recovery rules:

- Recovery starts automatically from `ExecutingMission` when `FaultPolicy` returns `StartRecovery`.
- Recovery success returns to the same mission segment.
- Recovery failure enters `FaultStopped`.
- Endpoint trigger during recovery is treated as a fault fact and should become P0 through policy.

## ThingsBoard Scope

ThingsBoard remains a cloud adapter. It must not own business state.

Retained RPCs:

- `clean_to_return`
- `clean_to_parking`
- `start_configured`
- `stop`
- `fault_reset`

Removed RPC:

- `reset` device reboot RPC

RPC behavior:

```text
ThingsBoard RPC callback
    -> RobotController::submit_command(CommandRequested)
    -> command is evaluated on the controller business thread
    -> bounded accepted or rejected response
    -> RPC reply
```

The RPC reply means only that the local business layer accepted or rejected the command. Final completion is reflected by telemetry snapshots.

`submit_command()` must not mutate state on the caller thread. It should enqueue a command request and wait for a bounded local acceptance result, for example by using a small completion object or promise owned by the request. If the controller queue does not answer before the local timeout, the RPC is rejected with a local timeout reason. This keeps cloud callbacks simple without giving them direct state-machine access.

Shared attributes:

- `schedules` update immediately through `SchedulerService`.
- Other runtime parameters are validated and saved as pending config, then promoted before the next task.

ThingsBoard uplink retained:

- startup attributes
- business telemetry
- health telemetry
- RPC reply

ThingsBoard uplink removed or not implemented:

- generic status event channel
- `shared_attr_update` status event
- `config_backup_fallback` status event
- high-frequency command event stream

## Health Telemetry

Health telemetry stays.

`HealthService` remains responsible for:

- production health payload
- diagnostics payload
- cloud telemetry upload through `CloudService`
- local JSONL rotating log

Health telemetry and business telemetry are different:

```text
business telemetry:
  RobotController snapshot, mission, state, fault, config version

health telemetry:
  motor, brush, BMS, IMU, GPS, diagnostics values
```

`FaultDetector` may read the same underlying device data as `HealthService`, but `HealthService` itself should not drive business state.

## Scheduler Scope

`SchedulerService` should only detect schedule windows.

When a window is hit:

```text
SchedulerService callback
    -> RobotController::post(ScheduleWindowHit)
```

The controller performs the same start gate as RPC start:

- state must be `Idle`
- low battery rejects start
- position must be acceptable
- config must be valid
- active fault must not exist

## Config Scope

Runtime config semantics:

- schedule changes take effect immediately
- all other runtime motion/task parameters take effect on next task

The controller should promote pending runtime config at mission start, not during an active mission.

Config update failures should be visible through logs and telemetry state, not a separate ThingsBoard status event stream.

## EventBus Scope

The existing synchronous `EventBus` may remain for non-business pub/sub if useful, but it must not directly mutate business state.

For business events, use `RobotController::post(...)`.

If an adapter still subscribes to `EventBus`, its callback should only post to `RobotController`.

## Testing Requirements

Unit tests:

- `RobotController` command transitions
- `RobotController` endpoint transitions
- `RobotController` stop behavior
- `RobotController` fault reset behavior
- `FaultPolicy` table decisions
- `FaultDetector` detection facts using mocks
- ThingsBoard RPC mapping to controller commands
- shared attributes immediate-vs-pending behavior
- Health telemetry builder still emits expected payloads

Threading tests:

- concurrent `post()` calls serialize correctly
- SafetyMonitor event adapter posts without directly changing state
- Watchdog timeout posts a fault fact
- RPC callback posts a command without directly calling state transition code

Hardware/integration tests:

- existing hardware system tests must keep equivalent behavior
- endpoint hard stop remains immediate
- full configured mission still closes through real limit sensors
- health JSONL still rotates and contains real diagnostics
- P0 fault chain enters `FaultStopped` and can be reset by `fault_reset`

## Acceptance Criteria

The refactor is accepted when:

- `RobotController` is the only owner of `RobotState`.
- `RobotController` is the only owner of active `MissionContext`.
- No external thread directly calls state-transition functions.
- P0 business decisions go through `FaultPolicy`.
- Hard endpoint stop still bypasses the business queue.
- ThingsBoard RPCs are limited to the retained command set.
- device reboot `reset` RPC is removed.
- generic ThingsBoard status event channel is removed.
- health telemetry and local JSONL remain working.
- low battery rejects start and does not enter `Charging`.
- `Charging` remains reserved but inactive.
- tests cover the business closure and hardware-equivalent flows.

## Implementation Order

1. Add `RobotController` event queue skeleton and snapshot API.
2. Move command handling from `RobotSupervisor` into `RobotController`.
3. Move state transitions from `RobotFsm` into simple controller handlers.
4. Add `FaultPolicy` and migrate P0/P1/P2 decisions.
5. Add `FaultDetector` and migrate runtime safety checks.
6. Replace `FaultHandler` and scattered `fault->report(P0, ...)` with controller events.
7. Rewire ThingsBoard RPCs to post controller commands.
8. Remove ThingsBoard reset/reboot RPC and generic status events.
9. Rewire scheduler, watchdog, safety monitor, recovery, and motion-start-failure paths to post events.
10. Preserve and validate health telemetry and local JSONL.
11. Delete obsolete compatibility code and redundant tests after equivalent coverage exists.
