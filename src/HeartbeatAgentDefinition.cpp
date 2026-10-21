#include "HeartbeatAgentDefinition.h"

#include "HeartbeatAgent.h"
#include "HeartbeatAgentConfig.h"

#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace pendarlab::sdk::mavlink_hub
{
  /// Parses an optional numeric field, returning true and writing out the value on success.
  template <typename T>
  static bool parseOptionalNumber(const std::unordered_map<std::string, std::string>& cfg, const std::string& key, T& out, std::vector<std::string>& messages, bool& hasError)
  {
    auto it = cfg.find(key);
    if (it == cfg.end()) {
      return true; // optional, absent is fine
    }
    try {
      // Parse as double first to handle any numeric text, then cast.
      const double value = std::stod(it->second);
      out = static_cast<T>(value);
      return true;
    } catch (const std::exception&) {
      messages.push_back("[HeartbeatAgent]: " + key + " is not a valid number: " + it->second);
      hasError = true;
      return false;
    }
  }

  /// Parses the raw config map into a HeartbeatAgentConfig, validating required fields and the
  /// optional heartbeat_freq and MAVLink field values. @param cfg the raw key/value config entries.
  /// @return a parse result with the parsed config on success, or populated error messages otherwise.
  AgentConfigParseResult HeartbeatAgentDefinition::parseConfig(const std::unordered_map<std::string, std::string>& cfg) const
  {
    AgentConfigParseResult result;
    auto config = std::make_unique<HeartbeatAgentConfig>();
    bool hasError = false;

    auto getRequired = [&cfg, &result, &hasError](const std::string& key) -> std::string {
      auto it = cfg.find(key);
      if (it == cfg.end()) {
        result.messages.push_back("[HeartbeatAgent]: required entry is missing: " + key);
        hasError = true;
        return "";
      }
      return it->second;
    };

    config->mavlink_endpoint_name = getRequired("mavlink_endpoint_name");
    if (config->mavlink_endpoint_name.empty()) {
      // mavlink_endpoint_name was found but empty; treat as a missing requirement.
      hasError = true;
    }

    auto freqIt = cfg.find("heartbeat_freq");
    if (freqIt != cfg.end()) {
      try {
        const double freq = std::stod(freqIt->second);
        if (freq <= 0.0) {
          result.messages.push_back("[HeartbeatAgent]: heartbeat_freq must be a positive number: " + freqIt->second);
          hasError = true;
        } else {
          config->heartbeat_freq = freq;
        }
      } catch (const std::exception&) {
        result.messages.push_back("[HeartbeatAgent]: heartbeat_freq is not a valid number: " + freqIt->second);
        hasError = true;
      }
    }

    parseOptionalNumber(cfg, "mavlink_sysid", config->mavlink_sysid, result.messages, hasError);
    parseOptionalNumber(cfg, "mavlink_compid", config->mavlink_compid, result.messages, hasError);
    parseOptionalNumber(cfg, "mavlink_type", config->mavlink_type, result.messages, hasError);
    parseOptionalNumber(cfg, "mavlink_autopilot", config->mavlink_autopilot, result.messages, hasError);
    parseOptionalNumber(cfg, "mavlink_base_mode", config->mavlink_base_mode, result.messages, hasError);
    parseOptionalNumber(cfg, "mavlink_custom_mode", config->mavlink_custom_mode, result.messages, hasError);
    parseOptionalNumber(cfg, "mavlink_system_status", config->mavlink_system_status, result.messages, hasError);

    if (!hasError) {
      result.parsed = std::move(config);
    }

    return result;
  }

  /// Creates a HeartbeatAgent from a parsed config. @param cfg the parsed config (must be a
  /// HeartbeatAgentConfig). @param requester handed to the agent. @return the new agent, or
  /// nullptr if cfg is not a HeartbeatAgentConfig.
  std::unique_ptr<Agent> HeartbeatAgentDefinition::create(const AgentConfig& cfg, std::unique_ptr<IManagerResourceRequester> requester) const
  {
    const auto* heartbeat_config = dynamic_cast<const HeartbeatAgentConfig*>(&cfg);
    if (heartbeat_config == nullptr) {
      return nullptr;
    }
    return std::make_unique<HeartbeatAgent>(*heartbeat_config, std::move(requester));
  }

  /// extern "C" factory entry point, dlsym'ed by mavlink_hub's LibLoader. @return a pointer
  /// to the singleton HeartbeatAgentDefinition.
  extern "C" const HeartbeatAgentDefinition* getHeartbeatAgentDefinition()
  {
    static HeartbeatAgentDefinition heartbeat_agent_def;
    return &heartbeat_agent_def;
  }
} // namespace pendarlab::sdk::mavlink_hub
