# Engine Control Refactor Design Note

## Goal

Refactor the main application so that:

1. Raw acquisition remains isolated to the ISR and processing path.
2. Filtering and decimation become the boundary between acquisition and all downstream consumers.
3. Websocket telemetry and SD logging no longer depend directly on the raw ping-pong buffer.
4. Ignition logic is easier to follow and owned by one control path.
5. Task notifications are reduced to the places where they are actually the right synchronization primitive.

---

## Current pain points

### 1. Raw-buffer ownership is unclear
The raw ping-pong buffer is currently involved in more than one logical path. That makes it harder to reason about races, especially now that collection has moved into the ISR.

### 2. `serverTask` is doing too much
`serverTask` currently acts as:
- the Mongoose polling task,
- a telemetry sender,
- a status-event sender,
- part of the ignition countdown flow.

This makes the transport layer responsible for application behavior.

### 3. Ignition state is split across multiple contexts
Ignition progression is currently spread across:
- `cmdHandlingTask`,
- the ignition software timer callback,
- `serverTask`.

That makes the sequence harder to verify and maintain.

### 4. Task notifications and flags are being used for mixed purposes
Some notifications represent real synchronization events, while others are effectively application-level messages. Mixing those patterns increases complexity.

### 5. Data-type and buffer-copy boundaries are not explicit enough
Raw ADC values, calibrated engineering values, telemetry payloads, and driver states should have clearly separate representations.

---

## Target architecture

### Ownership boundaries

#### ISR / acquisition path
Owns:
- SPI completion
- storing raw ADC samples
- advancing the active half-buffer
- notifying processing when a half-buffer is ready

Should **not** own:
- calibration
- telemetry formatting
- SD logging decisions
- websocket transmission

#### `processingTask`
Owns:
- consuming completed raw half-buffers
- CIC filtering / decimation
- calibration to engineering units
- generation of reduced-rate outputs
- publishing the latest processed telemetry snapshot
- appending processed records for SD logging

#### `serverTask`
Owns:
- `mg_mgr_poll()`
- sending the latest processed telemetry snapshot on schedule
- sending status / event messages that were prepared elsewhere

Should **not** own:
- raw buffer access
- ignition countdown progression
- actuator/control decisions

#### `cmdHandlingTask`
Owns:
- parsing incoming commands
- validating passwords / permissions
- actuating drivers
- arming / canceling ignition
- publishing command-result events

#### `shutdownTask`
Owns:
- orderly stop sequence
- final flush / close / unmount behavior
- cleanup of timers/tasks/resources

---

## Core refactor principle

The most important structural change is:

**The raw ping-pong buffer should terminate at `processingTask`.**

After CIC filtering and decimation are added:
- SD logging should consume processed decimated outputs.
- Websocket telemetry should consume a processed telemetry snapshot.
- `serverTask` should never touch raw acquisition memory.

This gives one clean boundary:

```text
ISR/raw samples -> processingTask -> decimated/calibrated outputs -> {SD logging, websocket telemetry}
```

---

## Recommended data flow

### Acquisition flow
1. ISR completes SPI transfer.
2. ISR stores raw samples into the active raw half-buffer.
3. When a half-buffer is complete, ISR notifies `processingTask`.
4. ISR switches to the other half-buffer.

### Processing flow
1. `processingTask` wakes on raw-buffer-ready notification.
2. It consumes the completed half-buffer.
3. It runs CIC filtering and decimation.
4. It calibrates the decimated channels.
5. It updates a `latest_telemetry_snapshot` structure.
6. It appends an SD log record if the SD cadence says one is due.

### Telemetry flow
1. A periodic timer or send tick tells `serverTask` it is time to transmit.
2. `serverTask` reads only `latest_telemetry_snapshot` and driver state.
3. `serverTask` serializes and sends the frame.

### Ignition flow
1. `cmdHandlingTask` accepts an ignition command.
2. It arms ignition state and starts a software timer.
3. The timer callback decrements the countdown and emits an ignition update event.
4. When the countdown reaches zero, the timer callback emits an ignition-fire event or directly triggers the ignition action.
5. `serverTask` only forwards prepared countdown/status messages to the client.

---

## Synchronization strategy

### Keep task notifications for these cases
Use task notifications only for real synchronization points such as:
- raw half-buffer ready,
- periodic telemetry send tick,
- shutdown requested,
- logging stop acknowledgment.

### Prefer an event queue for these cases
Use a small queue for discrete application-level messages such as:
- incorrect password,
- actuator changed,
- ignition countdown update,
- ignition started,
- ignition canceled,
- fault/shutdown event.

### Why this split helps
Task notifications are ideal when one task needs to wake another for a specific edge-triggered event.
Event queues are better when the content of the event matters and the server/UI path needs to forward or log it.

---

## Ignition refactor plan

### Keep
- The software timer approach.
- The idea that the countdown only needs to be synchronized with what the client sees, not hard real-time deterministic.

### Change
- Make one path own ignition progression.
- Do not let `serverTask` restart or advance the ignition timer.
- Make the timer callback responsible for the countdown state.

### Desired ownership

#### `cmdHandlingTask`
- validate ignition request,
- arm ignition,
- initialize countdown value,
- start/cancel timer.

#### Timer callback
- decrement countdown,
- publish countdown event,
- fire ignition event at zero,
- stop or disarm timer when complete.

#### `serverTask`
- forward the already-prepared countdown/status messages.

### Benefit
This removes the transport layer from the ignition state machine and makes the control sequence easier to reason about.

---

## Buffering recommendations

### Raw acquisition buffer
Because CIC decouples downstream consumers from the raw buffer, the raw ping-pong buffer may still be sufficient **if**:
- only the ISR produces raw samples,
- only `processingTask` consumes them,
- `processingTask` always finishes before the ISR wraps around.

### When triple buffering is needed
Move to a triple buffer only if:
- `processingTask` may occasionally fall behind acquisition,
- the ISR may need a spare buffer while a completed one is still waiting to be processed.

### Rule of thumb
- **Ping-pong is enough** for one producer and one consumer when the consumer always keeps up.
- **Triple buffering is safer** when completion latency can occasionally exceed one buffer interval.

### Recommendation for now
Do not add triple buffering immediately unless timing analysis shows `processingTask` can miss its consumption window.
First complete the decoupling so that raw buffers are only shared between ISR and `processingTask`.
Then evaluate whether ping-pong is still sufficient.

---

## Data structure cleanup

Create explicit representations for each stage of the pipeline.

### Raw sample buffer
Holds:
- raw ADC integer samples,
- acquisition-only data.

Example intent:
```c
uint16_t raw_sensor_buf[2][MAX_SENSOR_COUNT];
```

### Processed telemetry snapshot
Holds:
- calibrated engineering values,
- latest reduced-rate data intended for websocket transmission.

Example intent:
```c
typedef struct {
    float sensor_vals[MAX_SENSOR_COUNT];
    int driver_states[MAX_DRIVER_COUNT];
    uint32_t timestamp_ms;
} telemetry_snapshot_t;
```

### SD log record
Holds:
- timestamp,
- decimated/calibrated values,
- any metadata needed for logging.

### Ignition state
Holds:
- armed/disarmed status,
- countdown value,
- ignition-in-progress status,
- cancel/fault status.

### Benefit
These boundaries prevent accidental mixing of:
- raw integers,
- processed floats,
- driver state integers,
- formatted client strings.

---

## `serverTask` cleanup plan

### Keep in `serverTask`
- network polling,
- periodic send handling,
- formatting/sending telemetry payloads,
- forwarding event/status messages.

### Remove from `serverTask`
- raw buffer reads,
- ignition timer progression,
- direct application-state ownership,
- brittle special-case string offsets or shared log-string hacks.

### Desired model
`serverTask` should behave like a transport/output task, not like a second application controller.

---

## `processingTask` cleanup plan

Refactor `processingTask` into explicit internal phases.

### Suggested internal flow
1. wait for raw buffer ready,
2. consume raw buffer,
3. run CIC / decimation,
4. calibrate outputs,
5. update telemetry snapshot if due,
6. append SD log record if due,
7. handle shutdown/log-stop conditions.

### Benefit
This makes filtering and decimation the natural middle stage of the application instead of an added side feature.

---

## Command-handling cleanup plan

### Responsibilities to keep in `cmdHandlingTask`
- parse command payloads,
- validate passwords/authorization,
- perform driver actuation,
- arm/cancel ignition,
- publish a command-result event.

### Things to tighten
- define a clear backlog/overflow policy for queued commands,
- avoid relying on shared mutable strings for status propagation,
- make actuator changes emit structured events.

---

## Shutdown cleanup plan

### Desired shutdown order
1. stop or block new acquisition,
2. let `processingTask` finish any in-flight raw buffer,
3. flush pending processed SD records,
4. signal logging stopped,
5. close file / unmount storage,
6. stop/delete timers,
7. tear down tasks/resources.

### Goal
Shutdown should be deterministic and should not depend on ambiguous shared flags.

---

## Code hygiene checklist

### Ownership and architecture
- [ ] Make the raw ping-pong buffer private to ISR + `processingTask`.
- [ ] Remove all direct raw-buffer reads from `serverTask`.
- [ ] Add a `latest_telemetry_snapshot` owned by `processingTask`.
- [ ] Make SD logging consume processed decimated data only.
- [ ] Make websocket telemetry consume processed snapshot data only.

### Notifications and events
- [ ] Keep task notifications only for synchronization events.
- [ ] Add a small event queue for UI/status messages.
- [ ] Rename flags/notifications to reflect their exact purpose.

### Ignition
- [ ] Keep the software timer approach.
- [ ] Make the timer callback own countdown progression.
- [ ] Remove timer restart logic from `serverTask`.
- [ ] Make `cmdHandlingTask` responsible only for arm/cancel/reset.
- [ ] Ensure countdown reset is explicit for each new ignition command.
- [ ] Define the exact owner of the final ignition-fire action.

### Processing pipeline
- [ ] Implement the CIC and decimation stage in `processingTask`.
- [ ] Separate raw acquisition from processed outputs.
- [ ] Define SD cadence and websocket cadence from decimated outputs.
- [ ] Keep calibration after decimation unless a channel specifically requires pre-filter treatment.

### Buffers and types
- [ ] Separate raw sample buffers from float telemetry buffers.
- [ ] Separate driver-state buffers from float sensor-value buffers.
- [ ] Audit all `memcpy()` calls for correct source/destination element sizes.
- [ ] Remove obsolete globals tied to pre-CIC raw-buffer sharing.

### Server/output path
- [ ] Make `serverTask` transport-only.
- [ ] Replace shared-string/status hacks with structured event formatting.
- [ ] Serialize telemetry only from the processed snapshot.

### Command path
- [ ] Define queue overflow/backpressure behavior.
- [ ] Publish structured command-result events.
- [ ] Keep actuation and ignition arming in one command path.

### Shutdown
- [ ] Stop acquisition before final teardown.
- [ ] Flush processed log records before signaling stop complete.
- [ ] Delete or disarm ignition timer during shutdown.
- [ ] Make shutdown sequencing explicit.

---

## Recommended implementation order

### Phase 1: decouple raw acquisition from transport
- Remove raw-buffer reads from `serverTask`.
- Add a processed telemetry snapshot.
- Make `serverTask` send only from that snapshot.

### Phase 2: implement CIC / decimation
- Add CIC processing in `processingTask`.
- Make telemetry and SD outputs derive from decimated data.

### Phase 3: clean up ignition ownership
- Move countdown progression fully into the timer callback.
- Reduce `serverTask` to forwarding ignition events/messages only.

### Phase 4: replace mixed flags with cleaner messaging
- Keep task notifications for synchronization.
- Introduce an event queue for status/UI messages.

### Phase 5: reassess raw-buffer depth
- Measure worst-case `processingTask` latency.
- Keep ping-pong if it is sufficient.
- Move to triple buffering only if the processing path can lag by more than one buffer interval.

---

## Bottom line

The key change is not adding more tasks. The key change is making the filtering/decimation stage the formal handoff point between acquisition and everything else.

Once that boundary is enforced:
- the raw ping-pong buffer becomes simpler,
- `serverTask` becomes much cleaner,
- ignition logic becomes easier to understand,
- triple buffering becomes a performance decision instead of a structural band-aid.
