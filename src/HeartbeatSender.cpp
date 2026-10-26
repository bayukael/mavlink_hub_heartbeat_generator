#include "HeartbeatSender.h"

#include "HeartbeatMessageBuilder.h"
#include "PeriodicSender.h"

#include <mavlink_hub_sdk/mavlink_endpoint_user/IMavlinkEndpointUser.h>

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

  struct HeartbeatSender::Impl {
    HeartbeatAgentConfig config;
    std::unique_ptr<IManagerResourceRequester> requester;
    // Cache of per-endpoint users, keyed by endpoint name. Refreshed on each tick against
    // the resolved target set: entries no longer targeted are evicted, new targets request a user.
    std::unordered_map<std::string, std::unique_ptr<IMavlinkEndpointUser>> endpoint_users;
    // The inner periodic loop; wired below with the heartbeat builder and multi-target send.
    std::unique_ptr<PeriodicSender> periodic_sender;
  };

  /// Stores the config and requester and wires the internal periodic loop with a heartbeat
  /// message builder and a multi-target send function.
  HeartbeatSender::HeartbeatSender(const HeartbeatAgentConfig& config, std::unique_ptr<IManagerResourceRequester> requester)
      : d(std::make_unique<Impl>())
  {
    d->config = config;
    d->requester = std::move(requester);

    const int64_t period_us = static_cast<int64_t>(1'000'000.0 / config.heartbeat_freq);
    d->periodic_sender = std::make_unique<PeriodicSender>(
        [this]() {
          mavlink_message_t msg;
          makeHeartbeatMessage(d->config, msg);
          return msg;
        },
        [this](const mavlink_message_t& msg) {
          return dispatchToTargets(msg);
        },
        std::chrono::microseconds(period_us));
  }

  HeartbeatSender::~HeartbeatSender() = default;
  HeartbeatSender::HeartbeatSender(HeartbeatSender&&) noexcept = default;
  HeartbeatSender& HeartbeatSender::operator=(HeartbeatSender&&) noexcept = default;

  /// Resolves the current set of endpoint names to send a heartbeat to, according to the
  /// configured targeting mode:
  ///  - AllExcept: every endpoint reported by the requester minus the exclusion list.
  ///  - Listed: exactly the configured endpoints list.
  /// @return the target endpoint names in iteration order.
  std::vector<std::string> HeartbeatSender::resolveTargetNames()
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
  bool HeartbeatSender::dispatchToTargets(const mavlink_message_t& msg)
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

  /// Starts the internal periodic loop. @return true once running (false only if the inner
  /// sender could not be started, e.g. no requester to resolve targets).
  bool HeartbeatSender::start()
  {
    if (d->requester == nullptr || d->periodic_sender == nullptr) {
      return false;
    }
    return d->periodic_sender->start();
  }

  /// Stops the internal periodic loop.
  void HeartbeatSender::stop()
  {
    if (d->periodic_sender != nullptr) {
      d->periodic_sender->stop();
    }
  }

  /// @return true if the internal periodic loop is currently running.
  bool HeartbeatSender::isRunning() const
  {
    return d->periodic_sender != nullptr && d->periodic_sender->isRunning();
  }

  /// @return the loop period of the internal periodic sender.
  std::chrono::microseconds HeartbeatSender::period() const
  {
    return d->periodic_sender != nullptr ? d->periodic_sender->period() : std::chrono::microseconds(0);
  }
} // namespace pendarlab::app::mavlink_hub::plugin
