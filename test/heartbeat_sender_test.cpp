#include "HeartbeatSender.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

using pendarlab::sdk::mavlink_hub::HeartbeatSender;

namespace
{
  constexpr auto kWaitWindow = std::chrono::milliseconds(100);

  mavlink_message_t makeMessage()
  {
    mavlink_message_t msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.msgid = 0; // HEARTBEAT
    return msg;
  }
} // namespace

class HeartbeatSenderTest : public testing::Test
{
protected:
  void SetUp() override
  {
    sender.setPeriod(std::chrono::microseconds(10000)); // 100 Hz -> 10 ms period
    sender.setMessageBuilder([]() { return makeMessage(); });
    sender.setSendFn([&](const mavlink_message_t& msg) {
      sent_msgids.push_back(msg.msgid);
      ++send_count;
      return true;
    });
  }

  HeartbeatSender sender;
  std::atomic<int> send_count{0};
  std::vector<uint32_t> sent_msgids;
};

TEST_F(HeartbeatSenderTest, SendsAtConfiguredFrequency)
{
  sender.start();
  std::this_thread::sleep_for(kWaitWindow);
  sender.stop();

  // At 100 Hz over ~100 ms, expect ~10 sends; allow generous tolerance for scheduling.
  EXPECT_GE(send_count.load(), 5);
  EXPECT_LE(send_count.load(), 30);
}

TEST_F(HeartbeatSenderTest, SendsTheBuiltMessage)
{
  sender.setMessageBuilder([]() {
    mavlink_message_t msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.msgid = 0; // HEARTBEAT
    return msg;
  });

  sender.start();
  std::this_thread::sleep_for(kWaitWindow);
  sender.stop();

  ASSERT_GT(send_count.load(), 0);
  for (const auto msgid : sent_msgids) {
    EXPECT_EQ(msgid, 0); // every sent message is a HEARTBEAT
  }
}

TEST_F(HeartbeatSenderTest, NothingSentWithoutBuilder)
{
  sender.setMessageBuilder({});
  sender.setSendFn([&](const mavlink_message_t&) {
    ++send_count;
    return true;
  });

  sender.start();
  std::this_thread::sleep_for(kWaitWindow);
  sender.stop();

  EXPECT_EQ(send_count.load(), 0);
}

TEST_F(HeartbeatSenderTest, KeepsSendingOnSendFailure)
{
  sender.setSendFn([&](const mavlink_message_t&) {
    ++send_count;
    return false; // every send fails; the loop must keep running
  });

  sender.start();
  std::this_thread::sleep_for(kWaitWindow);
  sender.stop();

  EXPECT_GE(send_count.load(), 5);
}

TEST_F(HeartbeatSenderTest, StartIsNoOpWhenRunning)
{
  EXPECT_EQ(sender.start(), true);
  EXPECT_EQ(sender.start(), true); // no-op, still one thread
  std::this_thread::sleep_for(kWaitWindow);
  sender.stop();

  // If a second thread had been spawned, the count would be roughly doubled.
  EXPECT_LE(send_count.load(), 30);
}

TEST_F(HeartbeatSenderTest, StopIsNoOpWhenStopped)
{
  sender.stop(); // should not crash/hang when already stopped
  EXPECT_EQ(sender.isRunning(), false);
}

TEST_F(HeartbeatSenderTest, RepeatedStartStopCycles)
{
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(sender.start(), true);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    sender.stop();
    EXPECT_EQ(sender.isRunning(), false);
  }
}
