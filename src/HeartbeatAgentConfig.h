#pragma once

#include <mavlink_hub_sdk/agent/AgentConfig.h>

#include <cstdint>
#include <string>

namespace pendarlab::sdk::mavlink_hub
{
  /** Configuration for the heartbeat generator agent. Subclasses AgentConfig to add the
      fields needed to emit MAVLink HEARTBEAT messages to a Mavlink endpoint. */
  class HeartbeatAgentConfig : public AgentConfig
  {
  public:
    /// Name of the Mavlink endpoint to write HEARTBEAT messages to.
    std::string mavlink_endpoint_name;
    /// Frequency at which HEARTBEAT messages are emitted, in Hz. Defaults to 1.0.
    double heartbeat_freq = 1.0;
    /// MAVLink system id stamped on emitted HEARTBEAT messages. Defaults to 1.
    uint8_t mavlink_sysid = 1;
    /// MAVLink component id stamped on emitted HEARTBEAT messages. Defaults to 190 (onboard computer).
    uint8_t mavlink_compid = 190;
    /// MAVLink vehicle/component type (MAV_TYPE_*). Defaults to MAV_TYPE_ONBOARD_CONTROLLER.
    uint8_t mavlink_type = 43;
    /// MAVLink autopilot type (MAV_AUTOPILOT_*). Defaults to MAV_AUTOPILOT_INVALID.
    uint8_t mavlink_autopilot = 8;
    /// MAVLink base mode bitmap (MAV_MODE_FLAG_*). Defaults to 0.
    uint8_t mavlink_base_mode = 0;
    /// MAVLink custom mode bitfield. Defaults to 0.
    uint32_t mavlink_custom_mode = 0;
    /// MAVLink system status (MAV_STATE_*). Defaults to MAV_STATE_ACTIVE.
    uint8_t mavlink_system_status = 4;
  };
} // namespace pendarlab::sdk::mavlink_hub
