#pragma once

#include <mavlink_hub_sdk/agent/AgentDefinition.h>
#include <mavlink_hub_sdk/manager_resource_requester/IManagerResourceRequester.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace pendarlab::app::mavlink_hub::plugin
{
  using namespace pendarlab::sdk::mavlink_hub;

  /** Factory for HeartbeatAgent. Parses the plugin config and creates HeartbeatAgent instances. */
  class HeartbeatAgentDefinition : public AgentDefinition
  {
  public:
    /** Parses the string-keyed config map into a HeartbeatAgentConfig. @param cfg raw config entries.
        @return a parse result carrying the parsed config on success, or an error otherwise. */
    virtual AgentConfigParseResult parseConfig(const std::unordered_map<std::string, std::string>& cfg) const override;
    /** Creates a HeartbeatAgent from a parsed config. @param cfg the parsed config (must be a
        HeartbeatAgentConfig). @param requester handed to the agent for endpoint acquisition.
        @return the new agent, or nullptr if cfg is not a HeartbeatAgentConfig. */
    virtual std::unique_ptr<Agent> create(const AgentConfig& cfg, std::unique_ptr<IManagerResourceRequester> requester) const override;
  };
} // namespace pendarlab::app::mavlink_hub::plugin
