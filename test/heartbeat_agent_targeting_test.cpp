#include "HeartbeatAgent.h"
#include "HeartbeatAgentConfig.h"

#include <gtest/gtest.h>
#include <mavlink/common/mavlink.h>
#include <mavlink_hub_sdk/mavlink_endpoint_user/IMavlinkEndpointUser.h>
#include <mavlink_hub_sdk/manager_resource_requester/IManagerResourceRequester.h>

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace pendarlab::app::mavlink_hub::plugin
{
  namespace
  {
    using pendarlab::sdk::mavlink_hub::IManagerResourceRequester;
    using pendarlab::sdk::mavlink_hub::IMavlinkEndpointUser;

    constexpr auto kRunWindow = std::chrono::milliseconds(150);

    /// Shared per-endpoint write counter so multiple user instances aggregate the same count.
    struct WriteCounter
    {
      std::atomic<long> count{0};
    };

    /// A minimal IMavlinkEndpointUser whose writeMessage increments a shared counter.
    class MockEndpointUser : public IMavlinkEndpointUser
    {
    public:
      explicit MockEndpointUser(std::shared_ptr<WriteCounter> counter) : counter_(std::move(counter)) {}

      std::unique_ptr<pendarlab::lib::comm::MavlinkEndpointToken>
      createListener(std::function<void(const pendarlab::lib::comm::MavlinkEndpointPacket&)>) override
      {
        return nullptr;
      }
      std::optional<int> getNumOfListener() override { return 0; }
      std::optional<int> writeMessage(const mavlink_message_t&) override
      {
        const long n = ++counter_->count;
        return static_cast<int>(n);
      }
      std::optional<pendarlab::lib::comm::MavlinkEndpointState>
      getState() override
      {
        return pendarlab::lib::comm::MavlinkEndpointState::CONNECTED;
      }

    private:
      std::shared_ptr<WriteCounter> counter_;
    };

    /// A minimal IManagerResourceRequester backed by a register of endpoint names.
    /// requestMavlinkEndpoint returns a user only for registered names (mirroring the hub);
    /// getMavlinkEndpointList returns the registered set.
    class MockResourceRequester : public IManagerResourceRequester
    {
    public:
      std::vector<std::string> registered;
      std::unordered_map<std::string, std::shared_ptr<WriteCounter>> counters;

      std::unique_ptr<IMavlinkEndpointUser> requestMavlinkEndpoint(const std::string& name) override
      {
        for (const auto& reg : registered) {
          if (reg == name) {
            auto it = counters.find(name);
            if (it == counters.end()) {
              auto counter = std::make_shared<WriteCounter>();
              counters.emplace(name, counter);
              return std::make_unique<MockEndpointUser>(std::move(counter));
            }
            return std::make_unique<MockEndpointUser>(it->second);
          }
        }
        return nullptr; // not registered: skip silently
      }

      std::vector<std::string> getMavlinkEndpointList() override
      {
        return registered;
      }
    };

    /// Runs the agent for a short window and returns, per registered endpoint, how many
    /// messages it received. The requester is heap-allocated and owned by the agent (it is
    /// deleted with the agent at the end of this call), so the counts are copied out first.
    using CountMap = std::map<std::string, long>;

    CountMap runAgent(HeartbeatAgentConfig config, MockResourceRequester* requester, bool& started)
    {
      auto agent = std::make_unique<HeartbeatAgent>(config, std::unique_ptr<IManagerResourceRequester>(requester));
      started = agent->start();
      std::this_thread::sleep_for(kRunWindow);
      agent->stop();

      CountMap result;
      for (const auto& name : requester->registered) {
        auto it = requester->counters.find(name);
        result[name] = it == requester->counters.end() ? 0L : it->second->count.load();
      }
      return result;
    }
  } // namespace

  class HeartbeatAgentTargetingTest : public testing::Test
  {
  protected:
    HeartbeatAgentConfig makeConfig() const
    {
      HeartbeatAgentConfig config;
      config.heartbeat_freq = 100.0; // fast loop for tests
      return config;
    }
  };

  TEST_F(HeartbeatAgentTargetingTest, AllExceptTreatsEmptyExcludeAsAllConnected)
  {
    auto requester = std::make_unique<MockResourceRequester>();
    requester->registered = {"a", "b", "c"};

    auto config = makeConfig(); // targeting defaults to all_except with empty exclude
    bool started = false;
    auto counts = runAgent(config, requester.release(), started);

    EXPECT_TRUE(started);
    EXPECT_GT(counts["a"], 0L);
    EXPECT_GT(counts["b"], 0L);
    EXPECT_GT(counts["c"], 0L);
  }

  TEST_F(HeartbeatAgentTargetingTest, AllExceptExclusionsAreNotTargeted)
  {
    auto requester = std::make_unique<MockResourceRequester>();
    requester->registered = {"a", "b", "c"};

    auto config = makeConfig();
    config.targeting.mode = HeartbeatTargetingMode::AllExcept;
    config.targeting.exclude = {"b"};
    bool started = false;
    auto counts = runAgent(config, requester.release(), started);

    EXPECT_TRUE(started);
    EXPECT_GT(counts["a"], 0L);
    EXPECT_EQ(counts["b"], 0L);
    EXPECT_GT(counts["c"], 0L);
  }

  TEST_F(HeartbeatAgentTargetingTest, AllExceptUnknownExclusionIsSkippedSilently)
  {
    auto requester = std::make_unique<MockResourceRequester>();
    requester->registered = {"a", "b"};

    auto config = makeConfig();
    config.targeting.mode = HeartbeatTargetingMode::AllExcept;
    config.targeting.exclude = {"does_not_exist"};
    bool started = false;
    auto counts = runAgent(config, requester.release(), started);

    EXPECT_TRUE(started);
    EXPECT_GT(counts["a"], 0L);
    EXPECT_GT(counts["b"], 0L);
  }

  TEST_F(HeartbeatAgentTargetingTest, ListedTargetsExactlyTheListedEndpoints)
  {
    auto requester = std::make_unique<MockResourceRequester>();
    requester->registered = {"a", "b", "c"};

    auto config = makeConfig();
    config.targeting.mode = HeartbeatTargetingMode::Listed;
    config.targeting.endpoints = {"a", "c"};
    bool started = false;
    auto counts = runAgent(config, requester.release(), started);

    EXPECT_TRUE(started);
    EXPECT_GT(counts["a"], 0L);
    EXPECT_EQ(counts["b"], 0L);
    EXPECT_GT(counts["c"], 0L);
  }

  TEST_F(HeartbeatAgentTargetingTest, ListedEmptyListTargetsNoEndpoints)
  {
    auto requester = std::make_unique<MockResourceRequester>();
    requester->registered = {"a", "b", "c"};

    auto config = makeConfig();
    config.targeting.mode = HeartbeatTargetingMode::Listed;
    config.targeting.endpoints = {};
    bool started = false;
    auto counts = runAgent(config, requester.release(), started);

    EXPECT_TRUE(started);
    EXPECT_EQ(counts["a"], 0L);
    EXPECT_EQ(counts["b"], 0L);
    EXPECT_EQ(counts["c"], 0L);
  }

  TEST_F(HeartbeatAgentTargetingTest, ListedUnknownEndpointIsSkippedSilently)
  {
    auto requester = std::make_unique<MockResourceRequester>();
    requester->registered = {"a"};

    auto config = makeConfig();
    config.targeting.mode = HeartbeatTargetingMode::Listed;
    config.targeting.endpoints = {"a", "does_not_exist"};
    bool started = false;
    auto counts = runAgent(config, requester.release(), started);

    EXPECT_TRUE(started);
    EXPECT_GT(counts["a"], 0L);
    EXPECT_EQ(counts["does_not_exist"], 0L); // unknown name never requested -> 0
  }
} // namespace pendarlab::app::mavlink_hub::plugin
