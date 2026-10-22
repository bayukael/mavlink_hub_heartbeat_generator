#include "HeartbeatAgentConfig.h"
#include "HeartbeatMessageBuilder.h"

#include <gtest/gtest.h>
#include <mavlink/common/mavlink.h>

namespace pendarlab::app::mavlink_hub::plugin
{
  TEST(HeartbeatMessage, EmitsHeartbeatWithConfiguredFields)
  {
    HeartbeatAgentConfig config;
    config.mavlink_sysid = 42;
    config.mavlink_compid = 7;
    config.mavlink_type = MAV_TYPE_GCS;
    config.mavlink_autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    config.mavlink_base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    config.mavlink_custom_mode = 128;
    config.mavlink_system_status = MAV_STATE_ACTIVE;

    mavlink_message_t msg;
    makeHeartbeatMessage(config, msg);

    EXPECT_EQ(msg.msgid, MAVLINK_MSG_ID_HEARTBEAT);
    EXPECT_EQ(msg.sysid, 42);
    EXPECT_EQ(msg.compid, 7);

    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    EXPECT_EQ(heartbeat.type, MAV_TYPE_GCS);
    EXPECT_EQ(heartbeat.autopilot, MAV_AUTOPILOT_ARDUPILOTMEGA);
    EXPECT_EQ(heartbeat.base_mode, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
    EXPECT_EQ(heartbeat.custom_mode, 128);
    EXPECT_EQ(heartbeat.system_status, MAV_STATE_ACTIVE);
  }

  TEST(HeartbeatMessage, UsesDefaultsWhenNotConfigured)
  {
    HeartbeatAgentConfig config;

    mavlink_message_t msg;
    makeHeartbeatMessage(config, msg);

    EXPECT_EQ(msg.msgid, MAVLINK_MSG_ID_HEARTBEAT);
    EXPECT_EQ(msg.sysid, 255);
    EXPECT_EQ(msg.compid, 198);       // MAV_COMP_ID_PAIRING_MANAGER

    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    EXPECT_EQ(heartbeat.type, 6);            // MAV_TYPE_GCS
    EXPECT_EQ(heartbeat.autopilot, 8);       // MAV_AUTOPILOT_INVALID
    EXPECT_EQ(heartbeat.base_mode, 0);
    EXPECT_EQ(heartbeat.custom_mode, 0);
    EXPECT_EQ(heartbeat.system_status, 4);   // MAV_STATE_ACTIVE
  }
} // namespace pendarlab::app::mavlink_hub::plugin
