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
  } // namespace

  TEST(HeartbeatAgentDefinitionParseConfig, EmptyConfigParsesWithDefaultTargeting)
  {
    // A config without any targeting block defaults to mode all_except with empty exclude.
    auto result = def.parseConfig({});

    ASSERT_TRUE(result.ok());
    const auto* config = static_cast<const HeartbeatAgentConfig*>(result.parsed.get());
    EXPECT_EQ(config->targeting.mode, HeartbeatTargetingMode::AllExcept);
    EXPECT_TRUE(config->targeting.exclude.empty());
    EXPECT_TRUE(config->targeting.endpoints.empty());
  }

  TEST(HeartbeatAgentDefinitionParseConfig, AllExceptModeParses)
  {
    auto result = def.parseConfig({{"targeting.mode", "all_except"}});

    ASSERT_TRUE(result.ok());
    const auto* config = static_cast<const HeartbeatAgentConfig*>(result.parsed.get());
    EXPECT_EQ(config->targeting.mode, HeartbeatTargetingMode::AllExcept);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, ListedModeParses)
  {
    auto result = def.parseConfig({{"targeting.mode", "listed"}});

    ASSERT_TRUE(result.ok());
    const auto* config = static_cast<const HeartbeatAgentConfig*>(result.parsed.get());
    EXPECT_EQ(config->targeting.mode, HeartbeatTargetingMode::Listed);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, UnrecognizedModeRejected)
  {
    auto result = def.parseConfig({{"targeting.mode", "bogus"}});

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.parsed, nullptr);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, ExcludeListParsedWhenProvided)
  {
    auto cfg = std::unordered_map<std::string, std::string>{{"targeting.mode", "all_except"}};
    cfg["targeting.exclude"] = "pixhawk_serial, udp_link";
    auto result = def.parseConfig(cfg);

    ASSERT_TRUE(result.ok());
    const auto* config = static_cast<const HeartbeatAgentConfig*>(result.parsed.get());
    ASSERT_EQ(config->targeting.exclude.size(), 2u);
    EXPECT_EQ(config->targeting.exclude[0], "pixhawk_serial");
    EXPECT_EQ(config->targeting.exclude[1], "udp_link");
  }

  TEST(HeartbeatAgentDefinitionParseConfig, EndpointsListParsedWhenProvided)
  {
    auto cfg = std::unordered_map<std::string, std::string>{{"targeting.mode", "listed"}};
    cfg["targeting.endpoints"] = "ep_a,ep_b,ep_c";
    auto result = def.parseConfig(cfg);

    ASSERT_TRUE(result.ok());
    const auto* config = static_cast<const HeartbeatAgentConfig*>(result.parsed.get());
    ASSERT_EQ(config->targeting.endpoints.size(), 3u);
    EXPECT_EQ(config->targeting.endpoints[0], "ep_a");
    EXPECT_EQ(config->targeting.endpoints[1], "ep_b");
    EXPECT_EQ(config->targeting.endpoints[2], "ep_c");
  }

  TEST(HeartbeatAgentDefinitionParseConfig, EmptyListValuesYieldEmptyVectors)
  {
    auto cfg = std::unordered_map<std::string, std::string>{{"targeting.mode", "all_except"}};
    cfg["targeting.exclude"] = "";
    cfg["targeting.endpoints"] = " , ,";
    auto result = def.parseConfig(cfg);

    ASSERT_TRUE(result.ok());
    const auto* config = static_cast<const HeartbeatAgentConfig*>(result.parsed.get());
    EXPECT_TRUE(config->targeting.exclude.empty());
    EXPECT_TRUE(config->targeting.endpoints.empty());
  }

  TEST(HeartbeatAgentDefinitionParseConfig, ListsOnlyApplyThroughSelectedMode)
  {
    // Provided lists are always parsed; mode selects which is used at runtime.
    auto cfg = std::unordered_map<std::string, std::string>{{"targeting.mode", "all_except"}};
    cfg["targeting.exclude"] = "a";
    cfg["targeting.endpoints"] = "x,y";
    auto result = def.parseConfig(cfg);

    ASSERT_TRUE(result.ok());
    const auto* config = static_cast<const HeartbeatAgentConfig*>(result.parsed.get());
    EXPECT_EQ(config->targeting.exclude.size(), 1u);
    EXPECT_EQ(config->targeting.endpoints.size(), 2u);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, HeartbeatFreqDefaultsToOne)
  {
    auto result = def.parseConfig({});

    ASSERT_TRUE(result.ok());
    const auto* config = static_cast<const HeartbeatAgentConfig*>(result.parsed.get());
    EXPECT_DOUBLE_EQ(config->heartbeat_freq, 1.0);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, HeartbeatFreqParsedWhenProvided)
  {
    auto cfg = std::unordered_map<std::string, std::string>{{"heartbeat_freq", "10.0"}};
    auto result = def.parseConfig(cfg);

    ASSERT_TRUE(result.ok());
    const auto* config = static_cast<const HeartbeatAgentConfig*>(result.parsed.get());
    EXPECT_DOUBLE_EQ(config->heartbeat_freq, 10.0);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, HeartbeatFreqNonPositiveSetsParsedToNull)
  {
    auto result = def.parseConfig({{"heartbeat_freq", "0"}});

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.parsed, nullptr);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, HeartbeatFreqInvalidNumberSetsParsedToNull)
  {
    auto result = def.parseConfig({{"heartbeat_freq", "not_a_number"}});

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.parsed, nullptr);
  }

  TEST(HeartbeatAgentDefinitionParseConfig, MavlinkFieldsDefault)
  {
    auto result = def.parseConfig({});

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
    auto cfg = std::unordered_map<std::string, std::string>{};
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
    auto result = def.parseConfig({{"mavlink_sysid", "not_a_number"}});

    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.parsed, nullptr);
  }
} // namespace pendarlab::app::mavlink_hub::plugin
