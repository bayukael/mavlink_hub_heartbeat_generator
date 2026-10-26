#include "PeriodicSender.h"

#include <atomic>
#include <thread>
#include <utility>

namespace pendarlab::app::mavlink_hub::plugin
{
  struct PeriodicSender::PeriodicSenderImpl
  {
    std::thread thread;
    std::atomic<bool> stop{true};
    MessageBuilder message_builder;
    SendFn send_fn;
    std::chrono::microseconds period;

    /// The background loop: until stopped, build and send one message each period.
    void routine()
    {
      while (!stop.load()) {
        const auto next = std::chrono::steady_clock::now() + period;
        send_fn(message_builder());
        std::this_thread::sleep_until(next);
      }
    }
  };

  /// Stores the fixed wiring (builder, send fn, period).
  PeriodicSender::PeriodicSender(MessageBuilder builder, SendFn send, std::chrono::microseconds period)
      : d(std::make_unique<PeriodicSenderImpl>())
  {
    d->message_builder = std::move(builder);
    d->send_fn = std::move(send);
    d->period = period;
  }

  /// Stops the background loop if running, then destroys the sender.
  PeriodicSender::~PeriodicSender()
  {
    stop();
  }

  PeriodicSender::PeriodicSender(PeriodicSender&&) noexcept = default;
  PeriodicSender& PeriodicSender::operator=(PeriodicSender&&) noexcept = default;

  /// @return the loop period this sender was constructed with.
  std::chrono::microseconds PeriodicSender::period() const
  {
    return d->period;
  }

  /// Starts the background loop. @return true once running.
  bool PeriodicSender::start()
  {
    if (isRunning()) {
      return true;
    }
    d->stop.store(false);
    d->thread = std::thread(&PeriodicSenderImpl::routine, d.get());
    return true;
  }

  /// Stops the background loop, joining the thread.
  void PeriodicSender::stop()
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
  bool PeriodicSender::isRunning() const
  {
    return d->thread.joinable() && !d->stop.load();
  }
} // namespace pendarlab::app::mavlink_hub::plugin
