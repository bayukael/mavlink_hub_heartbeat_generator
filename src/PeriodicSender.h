#pragma once

#include <mavlink/common/mavlink.h>

#include <chrono>
#include <functional>
#include <memory>

namespace pendarlab::app::mavlink_hub::plugin
{
  /** A background thread that periodically invokes an injected message-builder and send
      function. The class owns the thread lifecycle and the timing. It is conversion-agnostic
      and MAVLink-agnostic: it only calls the message-builder and send callbacks, so it can be
      tested without a real MAVLink endpoint. All wiring (builder, send fn, period) is fixed at
      construction. */
  class PeriodicSender
  {
  public:
    /** Builds the message to send on each tick. Called once per period. */
    using MessageBuilder = std::function<mavlink_message_t()>;
    /** Sends a single message. The return value indicates whether the send succeeded and is
        otherwise ignored (the loop keeps running regardless). */
    using SendFn = std::function<bool(const mavlink_message_t&)>;

    /** Constructs a sender with its fixed wiring. @param builder produces the message each
        tick. @param send dispatch a single message. @param period time between ticks. */
    PeriodicSender(MessageBuilder builder, SendFn send, std::chrono::microseconds period);
    /// Stops the background loop (if running) and destroys the sender.
    ~PeriodicSender();
    /// Move constructor.
    PeriodicSender(PeriodicSender&&) noexcept;
    /// Move assignment.
    PeriodicSender& operator=(PeriodicSender&&) noexcept;

    /// @return the loop period this sender was constructed with.
    std::chrono::microseconds period() const;

    /** Starts the background loop. No-op if already running. @return true once running. */
    bool start();
    /** Stops the background loop (sets a stop flag and joins the thread). No-op if not running. */
    void stop();
    /// @return true if the background loop is currently running.
    bool isRunning() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> d;
  };
} // namespace pendarlab::app::mavlink_hub::plugin
