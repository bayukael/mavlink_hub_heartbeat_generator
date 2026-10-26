#pragma once

#include "HeartbeatAgentConfig.h"

#include <mavlink/common/mavlink.h>
#include <mavlink_hub_sdk/manager_resource_requester/IManagerResourceRequester.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace pendarlab::app::mavlink_hub::plugin
{
  using namespace pendarlab::sdk::mavlink_hub;

  /** MAVLink-aware heartbeat emitter. It owns the endpoint requester, the targeting config,
      and the per-endpoint user cache, and internally composes a PeriodicSender that
      periodically builds a HEARTBEAT message and dispatches it to the resolved target
      endpoint set (all_except or listed). The agent/config know nothing about MAVLink
      targeting; all of that lives here. */
  class HeartbeatSender
  {
  public:
    /** Constructs the sender, storing the config and requester and wiring an internal
        PeriodicSender built from the config's heartbeat_freq.
        @param config the heartbeat + targeting configuration.
        @param requester used to enumerate and obtain Mavlink endpoints. */
    HeartbeatSender(const HeartbeatAgentConfig& config, std::unique_ptr<IManagerResourceRequester> requester);
    /// Stops the background loop (if running) and destroys the sender.
    ~HeartbeatSender();
    /// Move constructor.
    HeartbeatSender(HeartbeatSender&&) noexcept;
    /// Move assignment.
    HeartbeatSender& operator=(HeartbeatSender&&) noexcept;

    /// Starts the internal periodic loop. No-op if already running. @return true once running.
    bool start();
    /// Stops the internal periodic loop. No-op if not running.
    void stop();
    /// @return true if the internal periodic loop is currently running.
    bool isRunning() const;
    /// @return the loop period of the internal periodic sender.
    std::chrono::microseconds period() const;

  private:
    /// Resolves the current target endpoint names per the configured targeting mode.
    std::vector<std::string> resolveTargetNames();
    /// Emits a heartbeat to every current target endpoint; returns true if at least one sent.
    bool dispatchToTargets(const mavlink_message_t& msg);

    struct Impl;
    std::unique_ptr<Impl> d;
  };
} // namespace pendarlab::app::mavlink_hub::plugin
