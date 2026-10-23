#include "HeartbeatAgent.h"

#include "HeartbeatMessageBuilder.h"
#include "HeartbeatSender.h"

#include <mavlink/common/mavlink.h>
#include <mavlink_hub_sdk/mavlink_endpoint_user/IMavlinkEndpointUser.h>
#include <mavlink_hub_sdk/manager_resource_requester/IManagerResourceRequester.h>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace pendarlab::app::mavlink_hub::plugin
{
  using namespace pendarlab::sdk::mavlink_hub;

  struct HeartbeatAgent::HeartbeatAgentImpl {
    HeartbeatAgentConfig config;

    std::unique_ptr<IManagerResourceRequester> requester;
    // Cache of per-endpoint users, keyed by endpoint name. Refreshed on each tick against
    // the resolved target set: entries no longer targeted are evicted, new targets request a user.
    std::unordered_map<std::string, std::unique_ptr<IMavlinkEndpointUser>> endpoint_users;

    HeartbeatSender heartbeat_sender;
  };

  /// Stores the config and requester, and configures the heartbeat loop period.
  HeartbeatAgent::HeartbeatAgent(const HeartbeatAgentConfig& config, std::unique_ptr<IManagerResourceRequester> requester)
      : d(std::make_unique<HeartbeatAgentImpl>())
  {
    d->config = config;
    d->requester = std::move(requester);

    const int64_t period_us = static_cast<int64_t>(1'000'000.0 / config.heartbeat_freq);
    d->heartbeat_sender.setPeriod(std::chrono::microseconds(period_us));
  }

  HeartbeatAgent::~HeartbeatAgent() = default;
  HeartbeatAgent::HeartbeatAgent(HeartbeatAgent&&) noexcept = default;
  HeartbeatAgent& HeartbeatAgent::operator=(HeartbeatAgent&&) noexcept = default;

  /// Resolves the current set of endpoint names to send a heartbeat to, according to the
  /// configured targeting mode:
  ///  - AllExcept: every endpoint reported by the requester minus the exclusion list.
  ///  - Listed: exactly the configured endpoints list.
  /// @return the target endpoint names in iteration order.
  std::vector<std::string> HeartbeatAgent::resolveTargetNames()
  {
    std::vector<std::string> targets;
    const HeartbeatTargeting& targeting = d->config.targeting;
    switch (targeting.mode) {
      case HeartbeatTargetingMode::AllExcept: {
        // Enumerate the registered endpoints (the hub returns the full registered set,
        // not filtered by connection state). Exclusions that match nothing are naturally skipped.
        const std::vector<std::string> all = d->requester->getMavlinkEndpointList();
        std::unordered_set<std::string> excluded(targeting.exclude.begin(), targeting.exclude.end());
        for (const auto& name : all) {
          if (!excluded.count(name)) {
            targets.push_back(name);
          }
        }
        break;
      }
      case HeartbeatTargetingMode::Listed:
        // Exactly the configured endpoints; names that are not registered are skipped
        // silently when the per-endpoint user request below returns nullptr.
        targets = targeting.endpoints;
        break;
    }
    return targets;
  }

  /// Emits a heartbeat to every current target endpoint. Unknown or unregistered endpoint
  /// names are skipped silently (their user request yields nullptr). A write that fails
  /// (e.g. to a disconnected endpoint) is also skipped silently, per the decided contract.
  /// @param msg the built HEARTBEAT message to dispatch. @return true if at least one target
  /// endpoint received the message.
  bool HeartbeatAgent::dispatchToTargets(const mavlink_message_t& msg)
  {
    const std::vector<std::string> targets = resolveTargetNames();
    std::unordered_set<std::string> wanted(targets.begin(), targets.end());

    // Evict cached users no longer in the target set.
    for (auto it = d->endpoint_users.begin(); it != d->endpoint_users.end();) {
      if (!wanted.count(it->first)) {
        it = d->endpoint_users.erase(it);
      } else {
        ++it;
      }
    }

    bool anySent = false;
    for (const auto& name : targets) {
      auto it = d->endpoint_users.find(name);
      if (it == d->endpoint_users.end()) {
        auto user = d->requester->requestMavlinkEndpoint(name);
        if (user == nullptr) {
          continue; // not a registered/requestable endpoint: skip silently
        }
        it = d->endpoint_users.emplace(name, std::move(user)).first;
      }
      // Per the decided contract, do not pre-check connection state: letting the write fail
      // is the cheaper path. A failed write simply does not count as sent.
      if (it->second->writeMessage(msg).has_value()) {
        anySent = true;
      }
    }
    return anySent;
  }

  /// Wires the heartbeat message builder + multi-endpoint dispatch and starts the background
  /// loop. @return true once running (false only if there is no requester to resolve targets).
  bool HeartbeatAgent::doStart()
  {
    if (d->requester == nullptr) {
      return false;
    }

    d->heartbeat_sender.setMessageBuilder([this]() {
      mavlink_message_t msg;
      makeHeartbeatMessage(d->config, msg);
      return msg;
    });
    d->heartbeat_sender.setSendFn([this](const mavlink_message_t& msg) {
      return dispatchToTargets(msg);
    });
    d->heartbeat_sender.start();

    return true;
  }

  /// Stops the heartbeat send-loop. @return true.
  bool HeartbeatAgent::doStop()
  {
    d->heartbeat_sender.stop();
    return true;
  }

  /// No-op configuration hook; the config is fixed at construction. @return true.
  bool HeartbeatAgent::doConfigure(const AgentConfig& config)
  {
    return true;
  }
} // namespace pendarlab::app::mavlink_hub::plugin
