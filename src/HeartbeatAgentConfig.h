#pragma once

#include <mavlink_hub_sdk/agent/AgentConfig.h>

#include <cstdint>
#include <string>
#include <vector>

namespace pendarlab::app::mavlink_hub::plugin
{
  using namespace pendarlab::sdk::mavlink_hub;

  /// Selects which endpoint list in HeartbeatTargeting determines the target set.
  enum class HeartbeatTargetingMode
  {
    AllExcept, ///< Target every registered endpoint except those in HeartbeatTargeting::exclude.
    Listed     ///< Target exactly the endpoints named in HeartbeatTargeting::endpoints.
  };

  /// The multi-endpoint targeting configuration of the heartbeat generator agent.
  struct HeartbeatTargeting
  {
    /// Which list applies. Defaults to AllExcept. Under AllExcept the exclusion list
    /// (exclude) applies; under Listed the endpoints list (endpoints) applies.
    HeartbeatTargetingMode mode = HeartbeatTargetingMode::AllExcept;
    /// Endpoint names to exclude when mode is AllExcept. An empty list targets every
    /// registered endpoint. Names that do not match a registered endpoint are skipped.
    std::vector<std::string> exclude;
    /// Endpoint names to target exactly when mode is Listed. An empty list targets no
    /// endpoints. Names that do not match a registered endpoint are skipped silently.
    std::vector<std::string> endpoints;
  };

  /** Configuration for the heartbeat generator agent. Subclasses AgentConfig to add the
      fields needed to emit MAVLink HEARTBEAT messages to selected Mavlink endpoints. */
  class HeartbeatAgentConfig : public AgentConfig
  {
  public:
    /// Multi-endpoint targeting: which connected endpoints receive heartbeats and how.
    HeartbeatTargeting targeting;
    /// Frequency at which HEARTBEAT messages are emitted, in Hz. Defaults to 1.0.
    double heartbeat_freq = 1.0;
    /// MAVLink system id stamped on emitted HEARTBEAT messages. Defaults to 255 to avoid
    /// colliding with autopilots that commonly use the default system id of 1.
    uint8_t mavlink_sysid = 255;
    /// MAVLink component id stamped on emitted HEARTBEAT messages. Defaults to 198 (MAV_COMP_ID_PAIRING_MANAGER).
    uint8_t mavlink_compid = 198;
    /// MAVLink vehicle/component type (MAV_TYPE_*). Defaults to MAV_TYPE_GCS.
    uint8_t mavlink_type = 6;
    /// MAVLink autopilot type (MAV_AUTOPILOT_*). Defaults to MAV_AUTOPILOT_INVALID.
    uint8_t mavlink_autopilot = 8;
    /// MAVLink base mode bitmap (MAV_MODE_FLAG_*). Defaults to 0.
    uint8_t mavlink_base_mode = 0;
    /// MAVLink custom mode bitfield. Defaults to 0.
    uint32_t mavlink_custom_mode = 0;
    /// MAVLink system status (MAV_STATE_*). Defaults to MAV_STATE_ACTIVE.
    uint8_t mavlink_system_status = 4;
  };
} // namespace pendarlab::app::mavlink_hub::plugin
