#pragma once

#include "HeartbeatAgentConfig.h"

#include <mavlink/common/mavlink.h>

namespace pendarlab::app::mavlink_hub::plugin
{
  /** Builds a MAVLink HEARTBEAT message from the given config. @param config the heartbeat
      parameters (sysid, compid, type, ...). @param msg filled with the encoded HEARTBEAT. */
  void makeHeartbeatMessage(const HeartbeatAgentConfig& config, mavlink_message_t& msg);
} // namespace pendarlab::app::mavlink_hub::plugin
