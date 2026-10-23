#include "HeartbeatAgentDefinition.h"

#include "HeartbeatAgent.h"
#include "HeartbeatAgentConfig.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <exception>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace pendarlab::app::mavlink_hub::plugin
{
  using namespace pendarlab::sdk::mavlink_hub;

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

  /// Splits a comma-separated list value into trimmed, non-empty entries. Empty tokens
  /// (e.g. from a trailing comma or whitespace-only entries) are dropped. A completely
  /// empty/absent value yields an empty list.
  static std::vector<std::string> splitCommaSeparated(const std::string& value)
  {
    std::vector<std::string> result;
    std::stringstream ss(value);
    std::string token;
    while (std::getline(ss, token, ',')) {
      token.erase(token.begin(), std::find_if(token.begin(), token.end(), [](unsigned char c) {
        return !std::isspace(c);
      }));
      token.erase(std::find_if(token.rbegin(), token.rend(), [](unsigned char c) {
        return !std::isspace(c);
      }).base(), token.end());
      if (!token.empty()) {
        result.push_back(token);
      }
    }
    return result;
  }

  /// Parses the raw config map into a HeartbeatAgentConfig, validating the optional
  /// heartbeat_freq, MAVLink field, and targeting values. The targeting block is encoded
  /// as flat keys (mavlink_hub feeds agents a flat string-map): \"targeting.mode\",
  /// \"targeting.exclude\" and \"targeting.endpoints\" (the two lists are comma-separated).
  /// @param cfg the raw key/value config entries.
  /// @return a parse result with the parsed config on success, or populated error messages otherwise.
  AgentConfigParseResult HeartbeatAgentDefinition::parseConfig(const std::unordered_map<std::string, std::string>& cfg) const
  {
    AgentConfigParseResult result;
    auto config = std::make_unique<HeartbeatAgentConfig>();
    bool hasError = false;

    // --- targeting block ---
    // mode: default all_except; reject any unrecognized value.
    auto modeIt = cfg.find("targeting.mode");
    if (modeIt != cfg.end()) {
      const std::string& mode = modeIt->second;
      if (mode == "all_except") {
        config->targeting.mode = HeartbeatTargetingMode::AllExcept;
      } else if (mode == "listed") {
        config->targeting.mode = HeartbeatTargetingMode::Listed;
      } else {
        result.messages.push_back("[HeartbeatAgent]: unrecognized targeting.mode: " + mode);
        hasError = true;
      }
    }
    // Lists: comma-separated, always parsed if present; which one applies is chosen by mode.
    auto excludeIt = cfg.find("targeting.exclude");
    if (excludeIt != cfg.end()) {
      config->targeting.exclude = splitCommaSeparated(excludeIt->second);
    }
    auto endpointsIt = cfg.find("targeting.endpoints");
    if (endpointsIt != cfg.end()) {
      config->targeting.endpoints = splitCommaSeparated(endpointsIt->second);
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
} // namespace pendarlab::app::mavlink_hub::plugin
