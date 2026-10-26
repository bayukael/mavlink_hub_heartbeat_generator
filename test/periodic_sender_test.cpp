#include "PeriodicSender.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

using pendarlab::app::mavlink_hub::plugin::PeriodicSender;

namespace
{
  constexpr auto kWaitWindow = std::chrono::milliseconds(100);
  constexpr auto kPeriod = std::chrono::microseconds(10000); // 100 Hz -> 10 ms period

  mavlink_message_t makeMessage()
  {
    mavlink_message_t msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.msgid = 0; // HEARTBEAT
    return msg;
  }
} // namespace

TEST(PeriodicSenderTest, SendsAtConfiguredFrequency)
{
  std::atomic<int> send_count{0};
  auto sender = PeriodicSender([]() { return makeMessage(); },
                               [&](const mavlink_message_t&) {
                                 ++send_count;
                                 return true;
                               },
                               kPeriod);

  sender.start();
  std::this_thread::sleep_for(kWaitWindow);
  sender.stop();

  // At 100 Hz over ~100 ms, expect ~10 sends; allow generous tolerance for scheduling.
  EXPECT_GE(send_count.load(), 5);
  EXPECT_LE(send_count.load(), 30);
}

TEST(PeriodicSenderTest, SendsTheBuiltMessage)
{
  std::vector<uint32_t> sent_msgids;
  auto sender = PeriodicSender([]() { return makeMessage(); },
                               [&](const mavlink_message_t& msg) {
                                 sent_msgids.push_back(msg.msgid);
                                 return true;
                               },
                               kPeriod);

  sender.start();
  std::this_thread::sleep_for(kWaitWindow);
  sender.stop();

  ASSERT_GT(sent_msgids.size(), 0u);
  for (const auto msgid : sent_msgids) {
    EXPECT_EQ(msgid, 0); // every sent message is a HEARTBEAT
  }
}

TEST(PeriodicSenderTest, KeepsSendingOnSendFailure)
{
  std::atomic<int> send_count{0};
  auto sender = PeriodicSender([]() { return makeMessage(); },
                               [&](const mavlink_message_t&) {
                                 ++send_count;
                                 return false; // every send fails; the loop must keep running
                               },
                               kPeriod);

  sender.start();
  std::this_thread::sleep_for(kWaitWindow);
  sender.stop();

  EXPECT_GE(send_count.load(), 5);
}

TEST(PeriodicSenderTest, StartIsNoOpWhenRunning)
{
  std::atomic<int> send_count{0};
  auto sender = PeriodicSender([]() { return makeMessage(); },
                               [&](const mavlink_message_t&) {
                                 ++send_count;
                                 return true;
                               },
                               kPeriod);

  EXPECT_EQ(sender.start(), true);
  EXPECT_EQ(sender.start(), true); // no-op, still one thread
  std::this_thread::sleep_for(kWaitWindow);
  sender.stop();

  // If a second thread had been spawned, the count would be roughly doubled.
  EXPECT_LE(send_count.load(), 30);
}

TEST(PeriodicSenderTest, StopIsNoOpWhenStopped)
{
  auto sender = PeriodicSender([]() { return makeMessage(); },
                               [](const mavlink_message_t&) { return true; },
                               kPeriod);

  sender.stop(); // should not crash/hang when already stopped
  EXPECT_EQ(sender.isRunning(), false);
}

TEST(PeriodicSenderTest, RepeatedStartStopCycles)
{
  std::atomic<int> send_count{0};
  auto sender = PeriodicSender([]() { return makeMessage(); },
                               [&](const mavlink_message_t&) {
                                 ++send_count;
                                 return true;
                               },
                               kPeriod);

  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(sender.start(), true);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    sender.stop();
    EXPECT_EQ(sender.isRunning(), false);
  }
}
