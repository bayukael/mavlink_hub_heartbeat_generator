# mavlink_hub_heartbeat_generator

A plugin for [mavlink_hub](https://github.com/bayukael/mavlink_hub) that periodically emits MAVLink `HEARTBEAT` messages to selected Mavlink endpoints at a configurable frequency.

The agent is loaded by `mavlink_hub` as a runtime plugin (`.so`). It exposes a single `HeartbeatAgent` that:

- writes MAVLink `HEARTBEAT` (id 0) messages to selected Mavlink endpoints on a background thread, and
- targets endpoints through a `targeting` block with two modes — `all_except` (every endpoint except an exclusion list) or `listed` (exactly a given list) — and
- emits at a configurable frequency (`heartbeat_freq`) with configurable MAVLink `sysid` / `compid` / message content fields.

## Overview / purpose

Autopilot software relies on MAVLink `HEARTBEAT` messages to recognize a transport as carrying a MAVLink device before it will send/receive through it. `mavlink_hub_heartbeat_generator` provides that by continuously emitting heartbeats to the selected Mavlink endpoints.

**Scope note.** Emitting heartbeats is a *necessary* condition for a MAVLink link to be established, but it is not sufficient on its own — other factors (autopilot configuration, transport behavior, system/component ids, etc.) also influence whether the link stays alive. The `link alive` outcome is an emergent, system-level property; this plugin's responsibility is limited to emitting heartbeats at a configurable frequency. Bundling this plugin with the rest of the agent/transport plugins into a package set is owned by the plugin-bundle task, not this repository.

## Data flow

```
HeartbeatAgent (background thread)
        │  every (1 / heartbeat_freq) seconds
        ▼
makeHeartbeatMessage(config) ──► mavlink_message_t (HEARTBEAT, id 0)
        │
        ▼ (for each target endpoint)
IMavlinkEndpointUser::writeMessage(msg) ──► MavlinkEndpoint ──► transport
```

The heartbeat loop runs on a background thread. Each tick it:

1. Resolves the current target endpoint set from the `targeting` configuration — `all_except` enumerates every endpoint reported by the manager (`IManagerResourceRequester::getMavlinkEndpointList()`) and removes the exclusion list; `listed` uses exactly the configured endpoint list.
2. Encodes a `mavlink_heartbeat_t` from the configured MAVLink fields (`sysid`, `compid`, `type`, `autopilot`, `base_mode`, `custom_mode`, `system_status`) via `makeHeartbeatMessage` (in `src/HeartbeatMessageBuilder.h/.cpp`).
3. Writes the resulting `mavlink_message_t` to each target endpoint via its `IMavlinkEndpointUser::writeMessage()`.

Endpoint names in `exclude`/`endpoints` that do not match a registered endpoint are skipped silently. A write to a disconnected endpoint fails silently (the endpoint is not counted as having received the message); the loop keeps running regardless.

`mavlink_version` is set by the MAVLink protocol on encode and is not configurable.

All plugin types (`HeartbeatAgent`, `HeartbeatAgentConfig`, `HeartbeatAgentDefinition`, `HeartbeatSender`, `PeriodicSender`, `makeHeartbeatMessage`) live in the `pendarlab::app::mavlink_hub::plugin` namespace, reflecting that this repository is an application-side mavlink_hub plugin rather than part of the SDK.

## Communication

The agent communicates with the autopilot exclusively through `pendarlab::sdk::mavlink_hub::IMavlinkEndpointUser`, which provides `writeMessage()`. `pendarlab::sdk::mavlink_hub::Agent` is designed so that the agent can obtain an `IMavlinkEndpointUser` by making a request through `IManagerResourceRequester`, which is provided by the mavlink_hub app when it calls `HeartbeatAgentDefinition::create()`.

## Building and installing

### Prerequisites

- CMake (>= 3.13) and a C++17 compiler.
- The `mavlink_hub_sdk` and its transitive dependencies installed and discoverable by CMake. **The SDK does not automatically deal with its own dependencies**, so you must set `CMAKE_PREFIX_PATH` so `find_package` can locate the SDK's transitive dependencies (e.g. `mavlink_endpoint`, ...). See "Configure and build" below.

### Configure and build

The SDK's CMake config pulls in its transitive dependencies, but it cannot resolve those on its own. You must set `CMAKE_PREFIX_PATH` to the install prefix(es) of the SDK and all its transitive dependencies (e.g. `mavlink_endpoint`, ...). There is no requirement that they share a common prefix.

From the project root (the directory containing this `README.md`):

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="<prefix(es) of the SDK and its dependencies>"
cmake --build build
```

The build produces a shared library in your build directory:

```
libmavlink_hub_heartbeat_generator.so
```

### Running the tests

The build compiles four GoogleTest executables:

```bash
ctest --test-dir build
# or run each directly:
./build/PeriodicSenderTest
./build/HeartbeatAgentParseConfigTest
./build/HeartbeatMessageTest
./build/HeartbeatAgentTargetingTest
```

`PeriodicSenderTest` verifies that heartbeat emission is *observable* at the configured frequency (it counts messages sent over a time window) rather than merely assumed. `HeartbeatAgentTargetingTest` verifies the `all_except` and `listed` targeting modes against a mocked manager (which endpoints receive heartbeats, including empty-list and unknown-name behavior).

### Making the plugin available to mavlink_hub

`mavlink_hub` loads agent plugins at runtime using `dlopen`, then looks up the exported factory symbol by name. This plugin exports:

```c
extern "C" const HeartbeatAgentDefinition* getHeartbeatAgentDefinition();
```

To make `mavlink_hub` able to load the `HeartbeatAgent`, provide `mavlink_hub` with:

1. The path to the built shared library — the `libmavlink_hub_heartbeat_generator.so` produced by the build (or a copy of it placed somewhere `mavlink_hub` can find).
2. The factory symbol name `getHeartbeatAgentDefinition`.
3. A name to register the agent under (used to reference this agent definition when spawning a `HeartbeatAgent`).

The agent's `extern "C"` entry point is in `src/HeartbeatAgentDefinition.cpp`.

## Configuration

When `mavlink_hub` spawns a `HeartbeatAgent`, it provides a set of string key/value options (a flat map, fed from the agent entry's JSON config) that are parsed by `HeartbeatAgentDefinition::parseConfig()`. Invalid values are reported as parse errors (the agent is not created).

### Targeting

The `targeting` block decides **which** endpoints receive heartbeats. `mavlink_hub` provides agent config as a flat string map, so the nested `targeting` object is encoded with dotted keys; list values are comma-separated strings.

| Key | Values | Default | Description |
| --- | --- | --- | --- |
| `targeting.mode` | `all_except` \| `listed` | `all_except` | Selects which list applies. Any other value is rejected at parse time. |
| `targeting.exclude` | comma-separated endpoint names | *(empty)* | Endpoints to **exclude** when mode is `all_except`. An empty list targets every registered endpoint. Used only in `all_except` mode. |
| `targeting.endpoints` | comma-separated endpoint names | *(empty)* | Endpoints to target **exactly** when mode is `listed`. An empty list targets no endpoints. Used only in `listed` mode. |

Notes:

- **No `targeting` block at all** defaults to `mode: all_except` with an empty `exclude` list — i.e. every registered endpoint is targeted. This is the default behavior for a config without targeting.
- In `all_except` mode the target set is evaluated every tick from the manager's current registered-endpoint list, so endpoints that connect after the agent starts are picked up automatically.
- The enumeration returns the manager's full set of **registered** endpoints (disconnected ones may still appear); a write to a disconnected endpoint fails silently rather than being pre-filtered.
- Endpoint names in `exclude`/`endpoints` that do not match a registered endpoint are skipped silently (no error, no warning).

### All fields

| Key | Default | Description |
| --- | --- | --- |
| `targeting.mode` | `all_except` | `all_except` or `listed`; header control for which list applies. |
| `targeting.exclude` | *(empty)* | Endpoint names to exclude (comma-separated) under `all_except`. Unknown names skipped. |
| `targeting.endpoints` | *(empty)* | Endpoint names to target exactly (comma-separated) under `listed`. Unknown names skipped. |
| `heartbeat_freq` | `1.0` | Frequency at which HEARTBEAT messages are emitted, in Hz. Must be a positive number. |
| `mavlink_sysid` | `255` | MAVLink system id stamped on emitted HEARTBEAT messages. Defaults to 255 to avoid colliding with autopilots that commonly use system id 1. |
| `mavlink_compid` | `198` | MAVLink component id stamped on emitted HEARTBEAT messages (198 = `MAV_COMP_ID_PAIRING_MANAGER`). |
| `mavlink_type` | `6` | MAVLink vehicle/component type (`MAV_TYPE_*`; 6 = `MAV_TYPE_GCS`). |
| `mavlink_autopilot` | `8` | MAVLink autopilot type (`MAV_AUTOPILOT_*`; 8 = `MAV_AUTOPILOT_INVALID`). |
| `mavlink_base_mode` | `0` | MAVLink system mode bitmap (`MAV_MODE_FLAG_*`). |
| `mavlink_custom_mode` | `0` | MAVLink custom mode bitfield (autopilot-specific). |
| `mavlink_system_status` | `4` | MAVLink system status (`MAV_STATE_*`; 4 = `MAV_STATE_ACTIVE`). |

## Configuration examples

The flat key/value form below is exactly the agent config `mavlink_hub` feeds the parser (keys map directly onto the agent entry's JSON config). They are ready to use as-is.

### Default: every endpoint (`all_except`, no exclusions)

```json
{
  "heartbeat_freq": "1"
}
```

This is equivalent to not supplying a `targeting` block at all: mode `all_except` with an empty `exclude` list, so a heartbeat is emitted once per second to **every** registered endpoint, with sysid 255, compid 198, type GCS, autopilot invalid, status active.

### `all_except`: all endpoints except two

```json
{
  "targeting.mode": "all_except",
  "targeting.exclude": "pixhawk_serial,udp_link",
  "heartbeat_freq": "5"
}
```

Emits a heartbeat to every registered endpoint **except** `pixhawk_serial` and `udp_link`. An exclusion that is not registered is skipped silently.

### `listed`: exactly two endpoints

```json
{
  "targeting.mode": "listed",
  "targeting.endpoints": "pixhawk_serial,udp_link",
  "heartbeat_freq": "2"
}
```

Emits a heartbeat only to `pixhawk_serial` and `udp_link`. An empty `targeting.endpoints` list targets no endpoints.

### Custom MAVLink identity

A GCS-style heartbeat with a custom system/component id sent to a single endpoint:

```json
{
  "targeting.mode": "listed",
  "targeting.endpoints": "udp_link",
  "heartbeat_freq": "2",
  "mavlink_sysid": "42",
  "mavlink_compid": "7",
  "mavlink_type": "6",
  "mavlink_autopilot": "3",
  "mavlink_base_mode": "1",
  "mavlink_custom_mode": "128",
  "mavlink_system_status": "4"
}
```

