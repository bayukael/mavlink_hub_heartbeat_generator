#pragma once

#include <mavlink_hub_sdk/agent/Agent.h>
#include <mavlink_hub_sdk/agent/AgentConfig.h>
#include <mavlink_hub_sdk/manager_resource_requester/IManagerResourceRequester.h>

#include "HeartbeatAgentConfig.h"
#include "HeartbeatSender.h"

#include <memory>

namespace pendarlab::app::mavlink_hub::plugin
{
  using namespace pendarlab::sdk::mavlink_hub;

  /** The heartbeat generator agent. A thin wrapper over HeartbeatSender: it owns the agent
      lifecycle (start/stop) and delegates the periodic multi-endpoint HEARTBEAT emission to
      the MAVLink-aware HeartbeatSender. */
  class HeartbeatAgent : public Agent
  {
  public:
    /** Constructs the agent, handing the config and requester to an internal HeartbeatSender.
        @param config the agent configuration.
        @param requester used to obtain the Mavlink endpoints at emit time. */
    HeartbeatAgent(const HeartbeatAgentConfig& config, std::unique_ptr<IManagerResourceRequester> requester);
    /// Destroys the agent, stopping all background activity.
    virtual ~HeartbeatAgent();
    /// Move constructor (the impl is a unique_ptr).
    HeartbeatAgent(HeartbeatAgent&&) noexcept;
    /// Move assignment (the impl is a unique_ptr).
    HeartbeatAgent& operator=(HeartbeatAgent&&) noexcept;

  protected:
    /// Starts the internal HeartbeatSender's periodic loop.
    virtual bool doStart() override;
    /// Stops the internal HeartbeatSender's periodic loop.
    virtual bool doStop() override;
    /// No-op configuration hook; configuration is fixed at construction.
    virtual bool doConfigure(const AgentConfig& config) override;

  private:
    struct HeartbeatAgentImpl;
    std::unique_ptr<HeartbeatAgentImpl> d;
  };
} // namespace pendarlab::app::mavlink_hub::plugin
