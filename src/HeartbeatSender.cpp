#include "HeartbeatSender.h"

#include <atomic>
#include <thread>

namespace pendarlab::app::mavlink_hub::plugin
{
  struct HeartbeatSender::Impl
  {
    std::thread thread;
    std::atomic<bool> stop{true};
    MessageBuilder message_builder;
    SendFn send_fn;
    std::chrono::microseconds period{std::chrono::microseconds(1000000)}; // 1 Hz default

    /// The background loop: until stopped, build and send one message each period.
    void routine()
    {
      while (!stop.load()) {
        const auto next = std::chrono::steady_clock::now() + period;
        if (message_builder && send_fn) {
          send_fn(message_builder());
        }
        std::this_thread::sleep_until(next);
      }
    }
  };

  /// Constructs an idle (not running) sender.
  HeartbeatSender::HeartbeatSender() : d(std::make_unique<Impl>()) {}

  /// Stops the background loop if running, then destroys the sender.
  HeartbeatSender::~HeartbeatSender()
  {
    stop();
  }

  HeartbeatSender::HeartbeatSender(HeartbeatSender&&) noexcept = default;
  HeartbeatSender& HeartbeatSender::operator=(HeartbeatSender&&) noexcept = default;

  /// Stores the message-builder callback. @param fn callback producing the message per tick.
  void HeartbeatSender::setMessageBuilder(MessageBuilder fn)
  {
    d->message_builder = std::move(fn);
  }

  /// Stores the send callback. @param fn callback dispatching a single message.
  void HeartbeatSender::setSendFn(SendFn fn)
  {
    d->send_fn = std::move(fn);
  }

  /// Sets the loop period. @param period time between ticks.
  void HeartbeatSender::setPeriod(std::chrono::microseconds period)
  {
    d->period = period;
  }

  /// @return the current loop period.
  std::chrono::microseconds HeartbeatSender::period() const
  {
    return d->period;
  }

  /// Starts the background loop. @return true once running.
  bool HeartbeatSender::start()
  {
    if (isRunning()) {
      return true;
    }
    d->stop.store(false);
    d->thread = std::thread(&Impl::routine, d.get());
    return true;
  }

  /// Stops the background loop, joining the thread.
  void HeartbeatSender::stop()
  {
    if (!isRunning()) {
      return;
    }
    d->stop.store(true);
    if (d->thread.joinable()) {
      d->thread.join();
    }
  }

  /// @return true if the background loop is currently running.
  bool HeartbeatSender::isRunning() const
  {
    return d->thread.joinable() && !d->stop.load();
  }
} // namespace pendarlab::app::mavlink_hub::plugin
