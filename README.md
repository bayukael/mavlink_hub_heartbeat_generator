# mavlink_hub_heartbeat_generator

A plugin for [mavlink_hub](https://github.com/bayukael/mavlink_hub) that periodically emits MAVLink `HEARTBEAT` messages to a Mavlink endpoint at a configurable frequency.

The agent is loaded by `mavlink_hub` as a runtime plugin (`.so`). It exposes a single `HeartbeatAgent` that:

- writes MAVLink `HEARTBEAT` (id 0) messages to a connected Mavlink endpoint on a background thread, and
- emits at a configurable frequency (`heartbeat_freq`) with configurable MAVLink `sysid` / `compid` / message content fields.

## Overview / purpose

Autopilot software relies on MAVLink `HEARTBEAT` messages to recognize a transport as carrying a MAVLink device before it will send/receive through it. `mavlink_hub_heartbeat_generator` provides that by continuously emitting heartbeats to a Mavlink endpoint.

**Scope note.** Emitting heartbeats is a *necessary* condition for a MAVLink link to be established, but it is not sufficient on its own — other factors (autopilot configuration, transport behavior, system/component ids, etc.) also influence whether the link stays alive. The `link alive` outcome is an emergent, system-level property; this plugin's responsibility is limited to emitting heartbeats at a configurable frequency. Bundling this plugin with the rest of the agent/transport plugins into a package set is owned by the plugin-bundle task, not this repository.

## Data flow

```
HeartbeatAgent (background thread)
        │  every (1 / heartbeat_freq) seconds
        ▼
makeHeartbeatMessage(config) ──► mavlink_message_t (HEARTBEAT, id 0)
        │
        ▼
IMavlinkEndpointUser::writeMessage(msg) ──► MavlinkEndpoint ──► transport
```

The heartbeat loop runs on a background thread. Each tick it:

1. Encodes a `mavlink_heartbeat_t` from the configured MAVLink fields (`sysid`, `compid`, `type`, `autopilot`, `base_mode`, `custom_mode`, `system_status`).
2. Writes the resulting `mavlink_message_t` to the Mavlink endpoint via `IMavlinkEndpointUser::writeMessage()`.

`mavlink_version` is set by the MAVLink protocol on encode and is not configurable.

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

The build compiles three GoogleTest executables:

```bash
ctest --test-dir build
# or run each directly:
./build/HeartbeatSenderTest
./build/HeartbeatAgentParseConfigTest
./build/HeartbeatMessageTest
```

`HeartbeatSenderTest` verifies that heartbeat emission is *observable* at the configured frequency (it counts messages sent over a time window) rather than merely assumed.

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

When `mavlink_hub` spawns a `HeartbeatAgent`, it provides a set of string key/value options that are parsed by `HeartbeatAgentDefinition::parseConfig()`. Invalid values are reported as parse errors (the agent is not created).

### Required fields

| Key | Description |
| --- | --- |
| `mavlink_endpoint_name` | Name of the Mavlink endpoint (registered in `mavlink_hub`) to write HEARTBEAT messages to. |

### Optional fields

| Key | Default | Description |
| --- | --- | --- |
| `heartbeat_freq` | `1.0` | Frequency at which HEARTBEAT messages are emitted, in Hz. Must be a positive number. |
| `mavlink_sysid` | `1` | MAVLink system id stamped on emitted HEARTBEAT messages. |
| `mavlink_compid` | `190` | MAVLink component id stamped on emitted HEARTBEAT messages (190 = onboard computer). |
| `mavlink_type` | `43` | MAVLink vehicle/component type (`MAV_TYPE_*`; 43 = `MAV_TYPE_ONBOARD_CONTROLLER`). |
| `mavlink_autopilot` | `8` | MAVLink autopilot type (`MAV_AUTOPILOT_*`; 8 = `MAV_AUTOPILOT_INVALID`). |
| `mavlink_base_mode` | `0` | MAVLink system mode bitmap (`MAV_MODE_FLAG_*`). |
| `mavlink_custom_mode` | `0` | MAVLink custom mode bitfield (autopilot-specific). |
| `mavlink_system_status` | `4` | MAVLink system status (`MAV_STATE_*`; 4 = `MAV_STATE_ACTIVE`). |

## Configuration examples

### Minimal configuration (required fields only)

```yaml
mavlink_endpoint_name: pixhawk_serial
```

This emits one HEARTBEAT per second with sysid 1, compid 190, type onboard-controller, autopilot invalid, status active.

### Non-default frequency

```yaml
mavlink_endpoint_name: pixhawk_serial
heartbeat_freq: 5
```

### Custom MAVLink identity

A GCS-style heartbeat with a custom system/component id:

```yaml
mavlink_endpoint_name: udp_link
heartbeat_freq: 2
mavlink_sysid: 42
mavlink_compid: 7
mavlink_type: 6            # MAV_TYPE_GCS
mavlink_autopilot: 3       # MAV_AUTOPILOT_ARDUPILOTMEGA
mavlink_base_mode: 1       # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
mavlink_custom_mode: 128
mavlink_system_status: 4   # MAV_STATE_ACTIVE
```
