#include "HeartbeatAgentConfig.h"
#include "HeartbeatAgentDefinition.h"

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>

namespace pendarlab::app::mavlink_hub::plugin
{
  namespace
  {
    HeartbeatAgentDefinition def;

    std::unordered_map<std::string, std::string> validBaseConfig()
    {
      return {
          {"mavlink_endpoint_name", "ep"},
      };
    }
  } // namespace

  TEST(HeartbeatAgentDefinitionParseConfig, MissingRequiredFieldSetsParsedToNull)
  {
    auto result = def.parseConfig({});

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.parsed, nullptr);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, EmptyEndpointNameSetsParsedToNull)
  {
    auto result = def.parseConfig({{"mavlink_endpoint_name", ""}});

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.parsed, nullptr);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, ValidConfigParses)
  {
    auto result = def.parseConfig(validBaseConfig());

    EXPECT_TRUE(result.ok());
    EXPECT_NE(result.parsed, nullptr);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, HeartbeatFreqDefaultsToOne)
  {
    auto result = def.parseConfig(validBaseConfig());

    ASSERT_TRUE(result.ok());
    const auto* config = static_cast<const HeartbeatAgentConfig*>(result.parsed.get());
    EXPECT_DOUBLE_EQ(config->heartbeat_freq, 1.0);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, HeartbeatFreqParsedWhenProvided)
  {
    auto cfg = validBaseConfig();
    cfg["heartbeat_freq"] = "10.0";
    auto result = def.parseConfig(cfg);

    ASSERT_TRUE(result.ok());
    const auto* config = static_cast<const HeartbeatAgentConfig*>(result.parsed.get());
    EXPECT_DOUBLE_EQ(config->heartbeat_freq, 10.0);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, HeartbeatFreqNonPositiveSetsParsedToNull)
  {
    auto cfg = validBaseConfig();
    cfg["heartbeat_freq"] = "0";
    auto result = def.parseConfig(cfg);

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.parsed, nullptr);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, HeartbeatFreqInvalidNumberSetsParsedToNull)
  {
    auto cfg = validBaseConfig();
    cfg["heartbeat_freq"] = "not_a_number";
    auto result = def.parseConfig(cfg);

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.parsed, nullptr);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, MavlinkFieldsDefault)
  {
    auto result = def.parseConfig(validBaseConfig());

    ASSERT_TRUE(result.ok());
    const auto* config = static_cast<const HeartbeatAgentConfig*>(result.parsed.get());
    EXPECT_EQ(config->mavlink_sysid, 255);
    EXPECT_EQ(config->mavlink_compid, 198);       // MAV_COMP_ID_PAIRING_MANAGER
    EXPECT_EQ(config->mavlink_type, 6);           // MAV_TYPE_GCS
    EXPECT_EQ(config->mavlink_autopilot, 8);      // MAV_AUTOPILOT_INVALID
    EXPECT_EQ(config->mavlink_base_mode, 0);
    EXPECT_EQ(config->mavlink_custom_mode, 0);
    EXPECT_EQ(config->mavlink_system_status, 4);  // MAV_STATE_ACTIVE
  }

  TEST(HeartbeatAgentDefinitionParseConfig, MavlinkFieldsParsedWhenProvided)
  {
    auto cfg = validBaseConfig();
    cfg["mavlink_sysid"] = "42";
    cfg["mavlink_compid"] = "7";
    cfg["mavlink_type"] = "6";       // MAV_TYPE_GCS
    cfg["mavlink_autopilot"] = "3";  // MAV_AUTOPILOT_ARDUPILOTMEGA
    cfg["mavlink_base_mode"] = "1";  // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    cfg["mavlink_custom_mode"] = "128";
    cfg["mavlink_system_status"] = "3"; // MAV_STATE_CRITICAL
    auto result = def.parseConfig(cfg);

    ASSERT_TRUE(result.ok());
    const auto* config = static_cast<const HeartbeatAgentConfig*>(result.parsed.get());
    EXPECT_EQ(config->mavlink_sysid, 42);
    EXPECT_EQ(config->mavlink_compid, 7);
    EXPECT_EQ(config->mavlink_type, 6);
    EXPECT_EQ(config->mavlink_autopilot, 3);
    EXPECT_EQ(config->mavlink_base_mode, 1);
    EXPECT_EQ(config->mavlink_custom_mode, 128);
    EXPECT_EQ(config->mavlink_system_status, 3);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, InvalidNumericFieldSetsParsedToNull)
  {
    auto cfg = validBaseConfig();
    cfg["mavlink_sysid"] = "not_a_number";
    auto result = def.parseConfig(cfg);

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.parsed, nullptr);
  }
} // namespace pendarlab::app::mavlink_hub::plugin
