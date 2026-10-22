#pragma once

#include <mavlink/common/mavlink.h>

#include <chrono>
#include <functional>
#include <memory>

namespace pendarlab::app::mavlink_hub::plugin
{
  /** A background thread that periodically emits one MAVLink message per tick through an
      injected send function. The class owns the thread lifecycle and the timing. It is
      conversion-agnostic: it only invokes the message-builder and send callbacks, so it can
      be tested without a real MAVLink endpoint. */
  class HeartbeatSender
  {
  public:
    /** Builds the message to send on each tick. Called once per period. */
    using MessageBuilder = std::function<mavlink_message_t()>;
    /** Sends a single message. The return value indicates whether the send succeeded and is
        otherwise ignored (the loop keeps running regardless). */
    using SendFn = std::function<bool(const mavlink_message_t&)>;

    /// Constructs an idle sender (not running).
    HeartbeatSender();
    /// Stops the background loop (if running) and destroys the sender.
    ~HeartbeatSender();
    /// Move constructor.
    HeartbeatSender(HeartbeatSender&&) noexcept;
    /// Move assignment.
    HeartbeatSender& operator=(HeartbeatSender&&) noexcept;

    /** Set the message-builder callback. Must be set BEFORE start(). @param fn callback
        producing the message to send each tick. */
    void setMessageBuilder(MessageBuilder fn);
    /** Set the send callback. Must be set BEFORE start(). @param fn callback dispatching a single message. */
    void setSendFn(SendFn fn);
    /** Set the loop period. Must be set BEFORE start(). @param period time between ticks. */
    void setPeriod(std::chrono::microseconds period);
    /// @return the current loop period.
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
