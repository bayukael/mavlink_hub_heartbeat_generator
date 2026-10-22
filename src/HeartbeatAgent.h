#pragma once

#include <mavlink_hub_sdk/agent/Agent.h>
#include <mavlink_hub_sdk/agent/AgentConfig.h>
#include <mavlink_hub_sdk/manager_resource_requester/IManagerResourceRequester.h>

#include "HeartbeatAgentConfig.h"

#include <memory>

namespace pendarlab::app::mavlink_hub::plugin
{
  using namespace pendarlab::sdk::mavlink_hub;

  /** The heartbeat generator agent. Emits MAVLink HEARTBEAT messages to a Mavlink endpoint at
      a configurable frequency on a background thread. */
  class HeartbeatAgent : public Agent
  {
  public:
    /** Constructs the agent, storing the config and the resource requester.
        @param config the agent configuration.
        @param requester used to obtain the Mavlink endpoint at start time. */
    HeartbeatAgent(const HeartbeatAgentConfig& config, std::unique_ptr<IManagerResourceRequester> requester);
    /// Destroys the agent, stopping all background activity.
    virtual ~HeartbeatAgent();
    /// Move constructor (the impl is a unique_ptr).
    HeartbeatAgent(HeartbeatAgent&&) noexcept;
    /// Move assignment (the impl is a unique_ptr).
    HeartbeatAgent& operator=(HeartbeatAgent&&) noexcept;

  protected:
    /// Requests the Mavlink endpoint, wires and starts the heartbeat send-loop.
    virtual bool doStart() override;
    /// Stops the heartbeat send-loop.
    virtual bool doStop() override;
    /// No-op configuration hook; configuration is fixed at construction.
    virtual bool doConfigure(const AgentConfig& config) override;

  private:
    struct HeartbeatAgentImpl;
    std::unique_ptr<HeartbeatAgentImpl> d;
  };
} // namespace pendarlab::app::mavlink_hub::plugin
