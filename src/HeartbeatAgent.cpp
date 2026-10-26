#include "HeartbeatAgent.h"

#include "HeartbeatSender.h"

#include <mavlink_hub_sdk/manager_resource_requester/IManagerResourceRequester.h>

#include <memory>
#include <utility>

namespace pendarlab::app::mavlink_hub::plugin
{
  using namespace pendarlab::sdk::mavlink_hub;

  struct HeartbeatAgent::HeartbeatAgentImpl {
    std::unique_ptr<HeartbeatSender> heartbeat_sender;
  };

  /// Stores the config/requester by building the MAVLink-aware HeartbeatSender.
  HeartbeatAgent::HeartbeatAgent(const HeartbeatAgentConfig& config, std::unique_ptr<IManagerResourceRequester> requester)
      : d(std::make_unique<HeartbeatAgentImpl>())
  {
    d->heartbeat_sender = std::make_unique<HeartbeatSender>(config, std::move(requester));
  }

  HeartbeatAgent::~HeartbeatAgent() = default;
  HeartbeatAgent::HeartbeatAgent(HeartbeatAgent&&) noexcept = default;
  HeartbeatAgent& HeartbeatAgent::operator=(HeartbeatAgent&&) noexcept = default;

  /// Starts the internal HeartbeatSender's periodic loop. @return false only if it cannot start.
  bool HeartbeatAgent::doStart()
  {
    return d->heartbeat_sender->start();
  }

  /// Stops the internal HeartbeatSender's periodic loop. @return true.
  bool HeartbeatAgent::doStop()
  {
    d->heartbeat_sender->stop();
    return true;
  }

  /// No-op configuration hook; the config is fixed at construction. @return true.
  bool HeartbeatAgent::doConfigure(const AgentConfig& config)
  {
    return true;
  }
} // namespace pendarlab::app::mavlink_hub::plugin
