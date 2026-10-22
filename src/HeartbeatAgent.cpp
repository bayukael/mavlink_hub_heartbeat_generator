#include "HeartbeatAgent.h"

#include "HeartbeatMessageBuilder.h"
#include "HeartbeatSender.h"

#include <mavlink/common/mavlink.h>
#include <mavlink_hub_sdk/mavlink_endpoint_user/IMavlinkEndpointUser.h>
#include <mavlink_hub_sdk/manager_resource_requester/IManagerResourceRequester.h>

#include <chrono>
#include <memory>

namespace pendarlab::app::mavlink_hub::plugin
{
  using namespace pendarlab::sdk::mavlink_hub;

  struct HeartbeatAgent::HeartbeatAgentImpl {
    HeartbeatAgentConfig config;

    std::unique_ptr<IManagerResourceRequester> requester;
    std::unique_ptr<IMavlinkEndpointUser> endpoint_user;

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

  /// Requests the Mavlink endpoint, wires the heartbeat message builder + send function, and
  /// starts the background loop. @return false if the endpoint is unavailable.
  bool HeartbeatAgent::doStart()
  {
    if (d->endpoint_user == nullptr) {
      d->endpoint_user = d->requester->requestMavlinkEndpoint(d->config.mavlink_endpoint_name);
      if (d->endpoint_user == nullptr) {
        return false;
      }
    }

    d->heartbeat_sender.setMessageBuilder([this]() {
      mavlink_message_t msg;
      makeHeartbeatMessage(d->config, msg);
      return msg;
    });
    d->heartbeat_sender.setSendFn([this](const mavlink_message_t& msg) {
      return d->endpoint_user != nullptr && d->endpoint_user->writeMessage(msg).has_value();
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
