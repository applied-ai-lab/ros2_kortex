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

// Soft stop from a ROS topic: the arm's own emergency stop, engaged and released by a std_msgs/Bool
// topic (hardware parameter `soft_stop_topic`; true = stop, false = release). This replaces the ROS 1
// kortex_driver's /in/emergency_stop and /in/clear_faults topics, which a ros2_control plugin cannot offer.
//
// Watchdog (hardware parameter soft_stop_watchdog, default on): silence on the topic for longer than
// soft_stop_timeout_s counts as a stop, so a soft stop that dies, or a lost network, stops the arm. The
// publisher must therefore send its state periodically (a heartbeat; Frank's soft stop does, at 10 Hz).
//
// Threading: the subscription callback (the controller manager's executor, not real time) only stores the
// requested state in an atomic; the real-time write() polls SoftStopLatch and performs the Kortex calls
// (ApplyEmergencyStop on engage, ClearFaults on release) once per change.

#ifndef KORTEX_DRIVER__SOFT_STOP_HPP_
#define KORTEX_DRIVER__SOFT_STOP_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp/subscription_base.hpp"

namespace kortex_driver
{

/// What the soft stop wants from the arm. ROS-free and lock-free: safe to use from the real-time loop.
///
/// - Engage once when a stop is requested and not yet engaged; Release once when the stop is lifted.
/// - Repeated messages (a soft stop republishing its state at 10 Hz) cause no further action.
/// - No message ever received: nothing happens (the topic is optional and starts unknown).
/// - After an engage, `holding()` stays true, also after release, until a controller switch: the arm
///   keeps its measured position instead of following controller commands, so a trajectory that kept
///   running during the stop can't make the arm jump when the stop is lifted.
/// - A failed Kortex call (`mark_failed()`) is retried after `retry_cycles` polls, not every cycle.
/// - Watchdog (`enable_watchdog(timeout)`, then `start(now)` on activation): no message for longer than
///   `timeout` since the last message (or since start) counts as a stop request.
class SoftStopLatch
{
public:
  using Clock = std::chrono::steady_clock;

  enum class Action
  {
    None,
    Engage,
    Release
  };

  explicit SoftStopLatch(unsigned retry_cycles = 100) : retry_cycles_(retry_cycles) {}

  /// Treat silence on the topic for longer than `timeout` as a stop (call before start()).
  void enable_watchdog(std::chrono::nanoseconds timeout)
  {
    watchdog_ = true;
    timeout_ns_ = timeout.count();
  }

  /// The arm is being activated: the watchdog's first deadline counts from now.
  void start(Clock::time_point now = Clock::now()) { last_message_ns_.store(ns(now)); }

  /// Any thread (subscription callback): the latest requested state, true = stop.
  void request(bool stop, Clock::time_point now = Clock::now())
  {
    requested_.store(stop);
    last_message_ns_.store(ns(now));
    received_.store(true);
  }

  /// The watchdog has expired: no message for longer than the timeout (false without a watchdog).
  bool timed_out(Clock::time_point now = Clock::now()) const
  {
    return watchdog_ && ns(now) - last_message_ns_.load() > timeout_ns_;
  }

  /// A stop is wanted: requested on the topic, or the watchdog expired.
  bool stop_wanted(Clock::time_point now = Clock::now()) const
  {
    return (received_.load() && requested_.load()) || timed_out(now);
  }

  /// Real-time thread: the action to perform now, if any.
  Action poll(Clock::time_point now = Clock::now())
  {
    if (!received_.load() && !timed_out(now))
    {
      return Action::None;
    }
    if (backoff_ > 0)
    {
      --backoff_;
      return Action::None;
    }
    const bool stop = stop_wanted(now);
    if (stop && !engaged_)
    {
      return Action::Engage;
    }
    if (!stop && engaged_)
    {
      return Action::Release;
    }
    return Action::None;
  }

  /// The emergency stop was applied.
  void mark_engaged()
  {
    engaged_ = true;
    holding_ = true;
  }

  /// The faults were cleared; the arm keeps holding until the next controller switch.
  void mark_released() { engaged_ = false; }

  /// The Kortex call for the last action failed: try again after retry_cycles polls.
  void mark_failed() { backoff_ = retry_cycles_; }

  /// A controller switch happened (perform_command_mode_switch): controllers may command again, unless
  /// the stop is still engaged or requested.
  void on_controller_switch(Clock::time_point now = Clock::now())
  {
    if (!engaged_ && !stop_wanted(now))
    {
      holding_ = false;
    }
  }

  bool engaged() const { return engaged_; }
  bool holding() const { return holding_; }
  bool watchdog() const { return watchdog_; }

private:
  static int64_t ns(Clock::time_point t)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
  }

  std::atomic<bool> requested_{false};
  std::atomic<bool> received_{false};
  bool engaged_ = false;
  bool holding_ = false;
  unsigned backoff_ = 0;
  const unsigned retry_cycles_;
  bool watchdog_ = false;
  int64_t timeout_ns_ = 0;
  std::atomic<int64_t> last_message_ns_{0};
};

/// Subscribe `latch` to a std_msgs/Bool soft-stop topic (true = stop). Reliable and transient local, so the
/// latest state of a latched publisher (e.g. Frank's /frank/soft_stop) arrives even if it was published
/// before this subscription existed.
rclcpp::SubscriptionBase::SharedPtr subscribe_soft_stop(
  rclcpp::Node & node, const std::string & topic, SoftStopLatch & latch);

}  // namespace kortex_driver

#endif  // KORTEX_DRIVER__SOFT_STOP_HPP_
