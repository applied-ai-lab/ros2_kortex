// Copyright 2026, Applied AI Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Soft stop without an arm: SoftStopLatch (what write() does on each cycle, including the watchdog) and the
// topic subscription (QoS compatible with a latched publisher such as Frank's /frank/soft_stop).

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#include "kortex_driver/soft_stop.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using kortex_driver::SoftStopLatch;
using Action = SoftStopLatch::Action;

// ---------------------------------------------------------------------------------------- latch

TEST(SoftStopLatch, NothingBeforeTheFirstMessage)
{
  SoftStopLatch latch;
  EXPECT_EQ(latch.poll(), Action::None);
  EXPECT_FALSE(latch.engaged());
  EXPECT_FALSE(latch.holding());
}

TEST(SoftStopLatch, ReleasedStateWithoutStopDoesNothing)
{
  SoftStopLatch latch;
  for (int i = 0; i < 10; ++i)  // a soft stop republishing "released" at 10 Hz
  {
    latch.request(false);
    EXPECT_EQ(latch.poll(), Action::None);
  }
}

TEST(SoftStopLatch, StopEngagesOnceAlthoughRepublished)
{
  SoftStopLatch latch;
  latch.request(true);
  EXPECT_EQ(latch.poll(), Action::Engage);
  latch.mark_engaged();
  EXPECT_TRUE(latch.engaged());
  EXPECT_TRUE(latch.holding());
  for (int i = 0; i < 10; ++i)
  {
    latch.request(true);
    EXPECT_EQ(latch.poll(), Action::None);
  }
}

TEST(SoftStopLatch, ReleaseOnceThenHoldUntilControllerSwitch)
{
  SoftStopLatch latch;
  latch.request(true);
  ASSERT_EQ(latch.poll(), Action::Engage);
  latch.mark_engaged();

  latch.on_controller_switch();  // switching while stopped does not end the hold
  EXPECT_TRUE(latch.holding());

  latch.request(false);
  EXPECT_EQ(latch.poll(), Action::Release);
  latch.mark_released();
  EXPECT_FALSE(latch.engaged());
  EXPECT_TRUE(latch.holding());  // released, but the arm keeps holding its position
  latch.request(false);
  EXPECT_EQ(latch.poll(), Action::None);

  latch.on_controller_switch();  // the operator (re)activates a controller
  EXPECT_FALSE(latch.holding());
}

TEST(SoftStopLatch, SwitchWhileStopRequestedButNotYetEngagedKeepsHold)
{
  SoftStopLatch latch;
  latch.request(true);
  ASSERT_EQ(latch.poll(), Action::Engage);
  latch.mark_engaged();
  latch.request(false);
  ASSERT_EQ(latch.poll(), Action::Release);
  latch.mark_released();
  latch.request(true);  // pressed again before anyone switched controllers
  latch.on_controller_switch();
  EXPECT_TRUE(latch.holding());
  EXPECT_EQ(latch.poll(), Action::Engage);  // and it engages again
}

TEST(SoftStopLatch, FailedCallIsRetriedAfterBackoff)
{
  SoftStopLatch latch(3);
  latch.request(true);
  ASSERT_EQ(latch.poll(), Action::Engage);
  latch.mark_failed();
  EXPECT_EQ(latch.poll(), Action::None);
  EXPECT_EQ(latch.poll(), Action::None);
  EXPECT_EQ(latch.poll(), Action::None);
  EXPECT_EQ(latch.poll(), Action::Engage);  // retried, not given up
  EXPECT_FALSE(latch.engaged());
}

TEST(SoftStopLatch, StopAndReleaseBetweenTwoCyclesDoesNothing)
{
  // Only the state at the cycle counts: a press and release between two 1 ms cycles is not an edge.
  SoftStopLatch latch;
  latch.request(true);
  latch.request(false);
  EXPECT_EQ(latch.poll(), Action::None);
}

// ----------------------------------------------------------------------------------- watchdog

namespace
{
using std::chrono::milliseconds;
const SoftStopLatch::Clock::time_point T0{std::chrono::seconds(1000)};

void watch(SoftStopLatch & latch)  // watchdog 500 ms, started at T0
{
  latch.enable_watchdog(milliseconds(500));
  latch.start(T0);
}
}  // namespace

TEST(SoftStopWatchdog, SilenceSinceStartEngages)
{
  // the soft stop never published (not running): fail-safe stop once the timeout passes
  SoftStopLatch latch;
  latch.enable_watchdog(milliseconds(500));
  latch.start(T0);
  EXPECT_EQ(latch.poll(T0 + milliseconds(400)), Action::None);
  EXPECT_FALSE(latch.timed_out(T0 + milliseconds(400)));
  EXPECT_TRUE(latch.timed_out(T0 + milliseconds(600)));
  EXPECT_EQ(latch.poll(T0 + milliseconds(600)), Action::Engage);
}

TEST(SoftStopWatchdog, HeartbeatKeepsTheArmRunning)
{
  SoftStopLatch latch;
  watch(latch);
  for (int i = 1; i <= 50; ++i)  // "released" every 100 ms for 5 s
  {
    latch.request(false, T0 + milliseconds(100 * i));
    EXPECT_EQ(latch.poll(T0 + milliseconds(100 * i + 50)), Action::None);
  }
  EXPECT_FALSE(latch.engaged());
}

TEST(SoftStopWatchdog, LostHeartbeatEngagesAndResumingReleases)
{
  SoftStopLatch latch;
  watch(latch);
  latch.request(false, T0 + milliseconds(100));
  EXPECT_EQ(latch.poll(T0 + milliseconds(500)), Action::None);  // 400 ms silent: still fine
  ASSERT_EQ(latch.poll(T0 + milliseconds(700)), Action::Engage);  // 600 ms silent: stop
  latch.mark_engaged();
  EXPECT_EQ(latch.poll(T0 + milliseconds(800)), Action::None);
  latch.on_controller_switch(T0 + milliseconds(800));
  EXPECT_TRUE(latch.holding());  // no un-holding while timed out

  latch.request(false, T0 + milliseconds(900));  // the soft stop is back, released
  ASSERT_EQ(latch.poll(T0 + milliseconds(950)), Action::Release);
  latch.mark_released();
  EXPECT_TRUE(latch.holding());  // holds until a controller is re-activated
  latch.on_controller_switch(T0 + milliseconds(960));
  EXPECT_FALSE(latch.holding());
}

TEST(SoftStopWatchdog, StopStaysEngagedWhenTheStopMessageStops)
{
  SoftStopLatch latch;
  watch(latch);
  latch.request(true, T0 + milliseconds(100));
  ASSERT_EQ(latch.poll(T0 + milliseconds(110)), Action::Engage);
  latch.mark_engaged();
  EXPECT_EQ(latch.poll(T0 + milliseconds(5000)), Action::None);  // silence while stopped: stays stopped
  EXPECT_TRUE(latch.engaged());
}

TEST(SoftStopWatchdog, OffMeansSilenceDoesNothing)
{
  SoftStopLatch latch;  // no enable_watchdog()
  latch.start(T0);
  EXPECT_FALSE(latch.timed_out(T0 + std::chrono::hours(1)));
  EXPECT_EQ(latch.poll(T0 + std::chrono::hours(1)), Action::None);
}

// --------------------------------------------------------------------------------- subscription

class SoftStopTopic : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    topic_ = "/test_soft_stop_" + std::to_string(getpid()) + "_" +
             ::testing::UnitTest::GetInstance()->current_test_info()->name();
    node_ = std::make_shared<rclcpp::Node>("test_soft_stop_node");
    executor_.add_node(node_);
  }

  void TearDown() override { executor_.remove_node(node_); }

  // The publisher Frank's soft stop uses: reliable, transient local, depth 1.
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr latched_publisher()
  {
    return node_->create_publisher<std_msgs::msg::Bool>(
      topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  }

  static std_msgs::msg::Bool msg(bool data)
  {
    std_msgs::msg::Bool m;
    m.data = data;
    return m;
  }

  bool spin_until(const std::function<bool()> & done)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < end)
    {
      executor_.spin_some(std::chrono::milliseconds(20));
      if (done())
      {
        return true;
      }
    }
    return false;
  }

  std::string topic_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(SoftStopTopic, LatchedStopPublishedBeforeTheDriverStartsEngages)
{
  auto pub = latched_publisher();
  pub->publish(msg(true));  // the soft stop is already pressed when the arm driver starts

  SoftStopLatch latch;
  auto sub = kortex_driver::subscribe_soft_stop(*node_, topic_, latch);
  Action action = Action::None;
  ASSERT_TRUE(spin_until([&] { return (action = latch.poll()) != Action::None; }));
  EXPECT_EQ(action, Action::Engage);
}

TEST_F(SoftStopTopic, FollowsStopAndRelease)
{
  SoftStopLatch latch;
  auto sub = kortex_driver::subscribe_soft_stop(*node_, topic_, latch);
  auto pub = latched_publisher();
  ASSERT_TRUE(spin_until([&] { return pub->get_subscription_count() > 0; }));

  pub->publish(msg(true));
  ASSERT_TRUE(spin_until([&] { return latch.poll() == Action::Engage; }));
  latch.mark_engaged();

  pub->publish(msg(false));
  ASSERT_TRUE(spin_until([&] { return latch.poll() == Action::Release; }));
  latch.mark_released();
  EXPECT_FALSE(latch.engaged());
}
