#include "HeartbeatMessageBuilder.h"

#include <mavlink/common/mavlink.h>

namespace pendarlab::app::mavlink_hub::plugin
{
  /// Encodes a MAVLink HEARTBEAT message from the config's fields.
  void makeHeartbeatMessage(const HeartbeatAgentConfig& config, mavlink_message_t& msg)
  {
    mavlink_heartbeat_t heartbeat{};
    heartbeat.custom_mode = config.mavlink_custom_mode;
    heartbeat.type = config.mavlink_type;
    heartbeat.autopilot = config.mavlink_autopilot;
    heartbeat.base_mode = config.mavlink_base_mode;
    heartbeat.system_status = config.mavlink_system_status;
    // mavlink_version is added by the protocol and is not writable.
    mavlink_msg_heartbeat_encode(config.mavlink_sysid, config.mavlink_compid, &msg, &heartbeat);
  }
} // namespace pendarlab::app::mavlink_hub::plugin
