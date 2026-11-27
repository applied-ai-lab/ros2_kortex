#include <gtest/gtest.h>
#include <limits>
#include "kortex_driver/hardware_interface.hpp"
namespace kortex_driver {
// Exercises mode validation and the actual cyclic command builder without connecting to an arm.
class KortexHardwareTestAccess {
public:
  static void setup(KortexMultiInterfaceHardware& h) {
    h.actuator_count_=7;
    h.start_torque_joints_.assign(7,false);h.torque_joints_.assign(7,true);
    h.effort_limits_.assign(7,5);h.last_effort_commands_.assign(7,0);
    h.arm_commands_efforts_.assign(7,100);h.arm_commands_positions_.assign(7,0);
    h.arm_commands_velocities_.assign(7,0);h.arm_efforts_.assign(7,0);
    h.arm_positions_.assign(7,0);h.arm_velocities_.assign(7,0);
    h.arm_joints_control_level_.resize(7);
    h.joint_calibration_.resize(7);h.effort_command_signs_.assign(7,1);
    h.arm_joint_names_.clear();h.info_.joints.clear();h.in_fault_=0;
    for(int i=0;i<7;++i){
      hardware_interface::ComponentInfo j;j.name="joint_"+std::to_string(i+1);
      h.info_.joints.push_back(j);h.arm_joint_names_.push_back(j.name);
      h.feedback_.add_actuators()->set_position(123+i);h.base_command_.add_actuators();
    }
  }
  static std::vector<std::string> effort(){std::vector<std::string> v;for(int i=1;i<=7;++i)v.push_back("joint_"+std::to_string(i)+"/effort");return v;}
  static bool blocked(KortexMultiInterfaceHardware& h){return h.block_write.load();}
  static void running(KortexMultiInterfaceHardware& h){h.joint_based_controller_running_=true;}
  static void unlimited(KortexMultiInterfaceHardware& h){h.effort_limits_[2]=INFINITY;}
  static void stop(KortexMultiInterfaceHardware& h){h.soft_stop_.request(true);}
  static void prepare(KortexMultiInterfaceHardware& h){h.use_effort_command_=true;h.prepareCommands();}
  static double command(KortexMultiInterfaceHardware& h,int i){return h.base_command_.actuators(i).torque_joint();}
  static void nan(KortexMultiInterfaceHardware& h){h.arm_commands_efforts_[0]=NAN;}
  static double position(KortexMultiInterfaceHardware& h){return h.base_command_.actuators(0).position();}
  static void readNoGripper(KortexMultiInterfaceHardware& h){h.has_gripper_=false;h.readGripperPosition();}
  static bool reseed(KortexMultiInterfaceHardware& h) {
    h.feedback_.mutable_actuators(0)->set_position(200);
    h.feedback_.mutable_actuators(0)->set_torque(3);
    const bool ok = h.seedArmState(h.feedback_);
    return ok && h.arm_commands_positions_[0] == h.arm_positions_[0] &&
      h.arm_efforts_[0] == -3 && h.arm_commands_efforts_[0] == -3;
  }
  static void torqueRunning(KortexMultiInterfaceHardware& h) {
    setup(h);h.use_effort_command_=true;h.joint_based_controller_running_=true;
    h.first_pass_=false;h.cyclic_async_=false;h.reset_fault_cmd_=NAN;
    h.arm_mode_=k_api::Base::LOW_LEVEL_SERVOING;
    h.feedback_.mutable_base()->set_active_state(k_api::Common::ARMSTATE_SERVOING_LOW_LEVEL);
    h.effort_disagreement_cycles_.assign(7,0);h.effort_startup_check_ms_=0;
  }
  static std::vector<std::string> positionInterfaces() {
    auto v=effort();for(auto& key:v)key.replace(key.find('/'),std::string::npos,"/position");return v;
  }
  static void observe(KortexMultiInterfaceHardware& h){h.updateFaultState();}
  static bool latched(const KortexMultiInterfaceHardware& h){return h.effort_fault_latched_;}
  static bool effortMode(const KortexMultiInterfaceHardware& h){return h.use_effort_command_;}
  static bool stopped(const KortexMultiInterfaceHardware& h){return h.soft_stop_.engaged();}
  static void requestStop(KortexMultiInterfaceHardware& h,bool stop){h.soft_stop_.request(stop);}
  static void fault(KortexMultiInterfaceHardware& h,int kind) {
    h.feedback_.mutable_base()->set_active_state(kind==1 ? k_api::Common::ARMSTATE_IN_FAULT :
      kind==4 ? k_api::Common::ARMSTATE_SERVOING_READY : k_api::Common::ARMSTATE_SERVOING_LOW_LEVEL);
    h.feedback_.mutable_actuators(2)->set_fault_bank_a(kind==2 || kind==5 ? 0x80000000u : 0);
    h.feedback_.mutable_actuators(2)->set_fault_bank_b(kind==3 || kind==5 ? 0x80000000u : 0);
  }
  static void moved(KortexMultiInterfaceHardware& h) {
    for(int i=0;i<7;++i)h.feedback_.mutable_actuators(i)->set_position(210+i);
  }
  static void staleCommands(KortexMultiInterfaceHardware& h) {
    h.arm_commands_positions_.assign(7,0);h.arm_commands_velocities_.assign(7,100);
    h.arm_commands_efforts_.assign(7,100);
  }
  static void positionMode(KortexMultiInterfaceHardware& h){h.use_effort_command_=false;}
  static void poweredStop(KortexMultiInterfaceHardware& h) { h.soft_stop_position_hold_=true; }
  static double holdingState(const KortexMultiInterfaceHardware& h) { return h.soft_stop_holding_state_; }
  static void expireWatchdog(KortexMultiInterfaceHardware& h) {
    h.soft_stop_.enable_watchdog(std::chrono::nanoseconds(1));
    h.soft_stop_.request(true, SoftStopLatch::Clock::now()-std::chrono::seconds(1));
  }
  static void reset(KortexMultiInterfaceHardware& h){h.fault_controller_running_=true;h.reset_fault_cmd_=1;}
  static bool resetSucceeded(const KortexMultiInterfaceHardware& h){return h.reset_fault_async_success_==1;}
  static k_api::BaseCyclic::Command frame(const KortexMultiInterfaceHardware& h){return h.base_command_;}
};

// Only the SDK boundary is simulated: read(), write(), mode prepare/perform, torque entry/exit,
// hold seeding and the command builder are the production implementations.
class SimulatedKortexHardware : public KortexMultiInterfaceHardware {
public:
  SimulatedKortexHardware(){KortexHardwareTestAccess::torqueRunning(*this);}
  k_api::ActuatorConfig::ControlMode actuator_mode=k_api::ActuatorConfig::TORQUE;
  bool fail_position=false,fail_frame=false,fault_on_refresh=false,fail_emergency=false;
  int emergency_stops=0,clear_faults=0;
  std::vector<k_api::ActuatorConfig::ControlMode> frame_modes;
  std::vector<k_api::BaseCyclic::Command> frames;
  std::vector<std::string> operations;
private:
  void refreshFeedback() override {
    if(fault_on_refresh)KortexHardwareTestAccess::fault(*this,1);
    KortexHardwareTestAccess::observe(*this);
  }
  void setServoingMode(k_api::Base::ServoingMode mode) override {
    operations.push_back(mode==k_api::Base::SINGLE_LEVEL_SERVOING ? "single" : "low");
  }
  void clearFaults() override {++clear_faults;KortexHardwareTestAccess::fault(*this,0);}
  bool applyEmergencyStop() override {++emergency_stops;return !fail_emergency;}
  bool setActuatorsControlMode(k_api::ActuatorConfig::ControlMode mode,
    const std::vector<bool>&) override {
    if(mode==k_api::ActuatorConfig::POSITION && fail_position)return false;
    operations.push_back(mode==k_api::ActuatorConfig::POSITION ? "position" : "torque");
    actuator_mode=mode;return true;
  }
  bool sendHoldFrame() override {if(fail_frame)return false;cyclicRefresh(true);return true;}
  void cyclicRefresh(bool command) override {
    if(command){frame_modes.push_back(actuator_mode);frames.push_back(KortexHardwareTestAccess::frame(*this));}
  }
};
static return_type readFeedback(KortexMultiInterfaceHardware& h) {
  return h.read(rclcpp::Time(0),rclcpp::Duration::from_seconds(.001));
}
static return_type writeCommands(KortexMultiInterfaceHardware& h) {
  return h.write(rclcpp::Time(0),rclcpp::Duration::from_seconds(.001));
}
TEST(EffortInterface, RefusedPrepareDoesNotBlockWrites) {
  KortexMultiInterfaceHardware h;KortexHardwareTestAccess::setup(h);KortexHardwareTestAccess::running(h);
  EXPECT_EQ(h.prepare_command_mode_switch(KortexHardwareTestAccess::effort(),{}),return_type::ERROR);
  EXPECT_FALSE(KortexHardwareTestAccess::blocked(h));
}
TEST(EffortInterface, RequiresFullArmLimitsAndReleasedStop) {
  KortexMultiInterfaceHardware h;KortexHardwareTestAccess::setup(h);
  EXPECT_EQ(h.prepare_command_mode_switch({"joint_1/effort"},{}),return_type::ERROR);
  EXPECT_EQ(h.prepare_command_mode_switch(KortexHardwareTestAccess::effort(),{}),return_type::OK);
  KortexHardwareTestAccess::unlimited(h);
  EXPECT_EQ(h.prepare_command_mode_switch(KortexHardwareTestAccess::effort(),{}),return_type::ERROR);
}
TEST(EffortInterface, StopAndSimultaneousTwistRefuseTorque) {
  KortexMultiInterfaceHardware h;KortexHardwareTestAccess::setup(h);
  auto v=KortexHardwareTestAccess::effort();v.push_back("tcp/twist.linear.x");
  EXPECT_EQ(h.prepare_command_mode_switch(v,{}),return_type::ERROR);
  KortexHardwareTestAccess::stop(h);
  EXPECT_EQ(h.prepare_command_mode_switch(KortexHardwareTestAccess::effort(),{}),return_type::ERROR);
  EXPECT_FALSE(KortexHardwareTestAccess::blocked(h));
}
TEST(EffortInterface, CyclicBuilderLimitsSlewAbsoluteTorqueAndNonfiniteCommands) {
  KortexMultiInterfaceHardware h;KortexHardwareTestAccess::setup(h);
  KortexHardwareTestAccess::prepare(h);EXPECT_NEAR(KortexHardwareTestAccess::command(h,0),.5,1e-7);
  EXPECT_DOUBLE_EQ(KortexHardwareTestAccess::position(h),123);
  for(int i=0;i<30;++i)KortexHardwareTestAccess::prepare(h);
  for(int i=0;i<7;++i)EXPECT_NEAR(KortexHardwareTestAccess::command(h,i),5,1e-7);
  KortexHardwareTestAccess::nan(h);KortexHardwareTestAccess::prepare(h);
  EXPECT_NEAR(KortexHardwareTestAccess::command(h,0),5,1e-7);
  EXPECT_NO_THROW(KortexHardwareTestAccess::readNoGripper(h));
}
TEST(EffortInterface, ReactivationReseedsMeasuredPoseAndRosTorque) {
  KortexMultiInterfaceHardware h;KortexHardwareTestAccess::setup(h);
  EXPECT_TRUE(KortexHardwareTestAccess::reseed(h));
}

TEST(EffortRecovery, ExternalClearCannotSendOldTorqueOrPositionCommands) {
  SimulatedKortexHardware h;
  KortexHardwareTestAccess::fault(h,1);
  ASSERT_EQ(readFeedback(h),return_type::OK);
  ASSERT_TRUE(KortexHardwareTestAccess::latched(h));
  ASSERT_EQ(writeCommands(h),return_type::OK);
  EXPECT_TRUE(h.frames.empty());
  EXPECT_EQ(h.clear_faults,0);  // A detected fault is never cleared automatically.

  KortexHardwareTestAccess::fault(h,0);  // Pendant/another client cleared it.
  KortexHardwareTestAccess::moved(h);
  ASSERT_EQ(readFeedback(h),return_type::OK);
  for(int cycle=0;cycle<5;++cycle) {
    KortexHardwareTestAccess::staleCommands(h);
    ASSERT_EQ(writeCommands(h),return_type::OK);
  }
  ASSERT_FALSE(h.frames.empty());
  ASSERT_GE(h.operations.size(),3u);
  EXPECT_EQ(h.operations[0],"single");
  EXPECT_EQ(h.operations[1],"position");
  EXPECT_EQ(h.operations[2],"low");
  for(std::size_t frame=0;frame<h.frames.size();++frame) {
    EXPECT_EQ(h.frame_modes[frame],k_api::ActuatorConfig::POSITION);
    for(int joint=0;joint<7;++joint)
      EXPECT_NEAR(h.frames[frame].actuators(joint).position(),210+joint,1e-4);
  }
  EXPECT_FALSE(KortexHardwareTestAccess::effortMode(h));
  EXPECT_TRUE(KortexHardwareTestAccess::latched(h));
  EXPECT_EQ(h.clear_faults,0);
}

class TorqueFaultSources : public testing::TestWithParam<int> {};
TEST_P(TorqueFaultSources, EveryLiveFaultSourceLatchesUntilPositionRecovery) {
  SimulatedKortexHardware h;
  KortexHardwareTestAccess::fault(h,GetParam());
  ASSERT_EQ(readFeedback(h),return_type::OK);
  ASSERT_TRUE(KortexHardwareTestAccess::latched(h));
  KortexHardwareTestAccess::fault(h,0);
  ASSERT_EQ(readFeedback(h),return_type::OK);
  EXPECT_TRUE(KortexHardwareTestAccess::latched(h));
}
INSTANTIATE_TEST_SUITE_P(BaseAndActuators,TorqueFaultSources,testing::Values(1,2,3,4,5));

TEST(EffortRecovery, PositionFaultDoesNotAcquireTorqueLatch) {
  SimulatedKortexHardware h;
  KortexHardwareTestAccess::positionMode(h);
  KortexHardwareTestAccess::fault(h,1);
  ASSERT_EQ(readFeedback(h),return_type::OK);
  EXPECT_FALSE(KortexHardwareTestAccess::latched(h));
}

TEST(EffortRecovery, RequiresSuccessfulFullPositionSwitchBeforeExplicitTorqueRestart) {
  SimulatedKortexHardware h;
  auto effort=KortexHardwareTestAccess::effort();
  auto position=KortexHardwareTestAccess::positionInterfaces();
  KortexHardwareTestAccess::fault(h,1);
  ASSERT_EQ(readFeedback(h),return_type::OK);
  KortexHardwareTestAccess::fault(h,0);
  ASSERT_EQ(readFeedback(h),return_type::OK);
  EXPECT_EQ(h.prepare_command_mode_switch(effort,effort),return_type::ERROR);
  EXPECT_EQ(h.prepare_command_mode_switch({"joint_1/position"},effort),return_type::ERROR);
  EXPECT_EQ(h.prepare_command_mode_switch({"joint_1/velocity"},effort),return_type::ERROR);
  EXPECT_EQ(h.prepare_command_mode_switch({"tcp/twist.linear.x"},effort),return_type::ERROR);

  // Loading a fault controller or stopping an arm controller must not re-arm torque.
  ASSERT_EQ(h.prepare_command_mode_switch({"reset_fault/command"},{}),return_type::OK);
  ASSERT_EQ(h.perform_command_mode_switch({},{}),return_type::OK);
  EXPECT_TRUE(KortexHardwareTestAccess::latched(h));
  KortexHardwareTestAccess::moved(h);
  ASSERT_EQ(h.prepare_command_mode_switch(position,effort),return_type::OK);
  ASSERT_EQ(h.perform_command_mode_switch(position,effort),return_type::OK);
  EXPECT_FALSE(KortexHardwareTestAccess::latched(h));
  EXPECT_FALSE(KortexHardwareTestAccess::effortMode(h));
  for(auto mode:h.frame_modes)EXPECT_EQ(mode,k_api::ActuatorConfig::POSITION);
  ASSERT_FALSE(h.frames.empty());
  EXPECT_NEAR(h.frames.back().actuators(0).position(),210,1e-4);

  // Only this separately requested activation may enter torque again.
  ASSERT_EQ(h.prepare_command_mode_switch(effort,position),return_type::OK);
  ASSERT_EQ(h.perform_command_mode_switch(effort,position),return_type::OK);
  EXPECT_TRUE(KortexHardwareTestAccess::effortMode(h));
  EXPECT_EQ(h.actuator_mode,k_api::ActuatorConfig::TORQUE);
  KortexHardwareTestAccess::staleCommands(h);
  ASSERT_EQ(writeCommands(h),return_type::OK);
  EXPECT_EQ(h.frame_modes.back(),k_api::ActuatorConfig::TORQUE);
  EXPECT_NEAR(h.frames.back().actuators(0).torque_joint(),.5,1e-7);
}

TEST(EffortRecovery, FaultAfterPrepareCannotEnterTorque) {
  SimulatedKortexHardware h;
  auto effort=KortexHardwareTestAccess::effort();
  ASSERT_EQ(h.prepare_command_mode_switch(effort,effort),return_type::OK);
  KortexHardwareTestAccess::fault(h,1);
  ASSERT_EQ(readFeedback(h),return_type::OK);
  KortexHardwareTestAccess::fault(h,0);
  ASSERT_EQ(readFeedback(h),return_type::OK);
  EXPECT_EQ(h.perform_command_mode_switch(effort,effort),return_type::ERROR);
  EXPECT_TRUE(h.frames.empty());
  EXPECT_FALSE(KortexHardwareTestAccess::blocked(h));
  EXPECT_TRUE(KortexHardwareTestAccess::latched(h));
}

class FailedTorqueRecovery : public testing::TestWithParam<int> {};
TEST_P(FailedTorqueRecovery, FailedRestorationStopsAndDoesNotClearLatch) {
  SimulatedKortexHardware h;
  KortexHardwareTestAccess::fault(h,1);
  ASSERT_EQ(readFeedback(h),return_type::OK);
  KortexHardwareTestAccess::fault(h,0);
  ASSERT_EQ(readFeedback(h),return_type::OK);
  h.fail_position=GetParam()==0;
  h.fail_frame=GetParam()==1;
  h.fault_on_refresh=GetParam()==2;
  EXPECT_EQ(writeCommands(h),return_type::ERROR);
  EXPECT_GT(h.emergency_stops,0);
  EXPECT_TRUE(KortexHardwareTestAccess::stopped(h));
  EXPECT_TRUE(KortexHardwareTestAccess::latched(h));
  for(auto mode:h.frame_modes)EXPECT_EQ(mode,k_api::ActuatorConfig::POSITION);
}
INSTANTIATE_TEST_SUITE_P(ModeFrameAndNewFault,FailedTorqueRecovery,testing::Values(0,1,2));

TEST(EffortRecovery, FailedExplicitPositionSwitchDoesNotRearmTorque) {
  SimulatedKortexHardware h;
  KortexHardwareTestAccess::fault(h,1);
  ASSERT_EQ(readFeedback(h),return_type::OK);
  KortexHardwareTestAccess::fault(h,0);
  ASSERT_EQ(readFeedback(h),return_type::OK);
  auto position=KortexHardwareTestAccess::positionInterfaces();
  auto effort=KortexHardwareTestAccess::effort();
  ASSERT_EQ(h.prepare_command_mode_switch(position,effort),return_type::OK);
  h.fail_position=true;
  EXPECT_EQ(h.perform_command_mode_switch(position,effort),return_type::ERROR);
  EXPECT_TRUE(KortexHardwareTestAccess::latched(h));
  EXPECT_TRUE(KortexHardwareTestAccess::stopped(h));
  EXPECT_FALSE(KortexHardwareTestAccess::blocked(h));
  EXPECT_TRUE(h.frames.empty());
}

TEST(EffortRecovery, FaultFirstObservedDuringTorqueExitCannotSendAnotherTorqueFrame) {
  SimulatedKortexHardware h;
  auto position=KortexHardwareTestAccess::positionInterfaces();
  auto effort=KortexHardwareTestAccess::effort();
  ASSERT_EQ(h.prepare_command_mode_switch(position,effort),return_type::OK);
  h.fault_on_refresh=true;
  EXPECT_EQ(h.perform_command_mode_switch(position,effort),return_type::ERROR);
  EXPECT_TRUE(KortexHardwareTestAccess::latched(h));
  EXPECT_TRUE(KortexHardwareTestAccess::stopped(h));
  EXPECT_TRUE(h.frames.empty());
}

TEST(EffortRecovery, ExplicitFaultResetAlsoRequiresPositionSwitch) {
  SimulatedKortexHardware h;
  KortexHardwareTestAccess::reset(h);
  ASSERT_EQ(writeCommands(h),return_type::OK);
  EXPECT_TRUE(KortexHardwareTestAccess::resetSucceeded(h));
  EXPECT_EQ(h.clear_faults,1);
  EXPECT_TRUE(KortexHardwareTestAccess::latched(h));
  EXPECT_FALSE(KortexHardwareTestAccess::effortMode(h));
  for(auto mode:h.frame_modes)EXPECT_EQ(mode,k_api::ActuatorConfig::POSITION);
}

TEST(EffortRecovery, SoftStopReleaseDoesNotClearExistingTorqueFaultLatch) {
  SimulatedKortexHardware h;
  KortexHardwareTestAccess::fault(h,1);
  ASSERT_EQ(readFeedback(h),return_type::OK);
  KortexHardwareTestAccess::requestStop(h,true);
  ASSERT_EQ(writeCommands(h),return_type::OK);
  EXPECT_TRUE(h.frames.empty());
  KortexHardwareTestAccess::requestStop(h,false);
  ASSERT_EQ(writeCommands(h),return_type::OK);
  EXPECT_FALSE(KortexHardwareTestAccess::stopped(h));
  EXPECT_TRUE(KortexHardwareTestAccess::latched(h));
  EXPECT_FALSE(KortexHardwareTestAccess::effortMode(h));
  EXPECT_EQ(h.clear_faults,1);
  for(auto mode:h.frame_modes)EXPECT_EQ(mode,k_api::ActuatorConfig::POSITION);
}
}

namespace kortex_driver {
TEST(EffortInterface, PoweredSoftStopHoldsCapturedPoseUntilExplicitReactivation) {
  SimulatedKortexHardware h;
  KortexHardwareTestAccess::poweredStop(h);
  KortexHardwareTestAccess::requestStop(h,true);
  ASSERT_EQ(writeCommands(h),return_type::OK);
  EXPECT_EQ(h.emergency_stops,0);
  EXPECT_EQ(h.actuator_mode,k_api::ActuatorConfig::POSITION);
  EXPECT_DOUBLE_EQ(KortexHardwareTestAccess::holdingState(h),1);
  ASSERT_FALSE(h.frames.empty());
  const auto captured=h.frames.back();
  KortexHardwareTestAccess::moved(h);
  KortexHardwareTestAccess::staleCommands(h);
  ASSERT_EQ(writeCommands(h),return_type::OK);
  for(int i=0;i<7;++i) EXPECT_FLOAT_EQ(h.frames.back().actuators(i).position(),captured.actuators(i).position());
  EXPECT_EQ(h.prepare_command_mode_switch(KortexHardwareTestAccess::positionInterfaces(),KortexHardwareTestAccess::effort()),return_type::ERROR);
  KortexHardwareTestAccess::requestStop(h,false);
  ASSERT_EQ(writeCommands(h),return_type::OK);
  EXPECT_EQ(h.clear_faults,0);
  EXPECT_DOUBLE_EQ(KortexHardwareTestAccess::holdingState(h),1);
  for(int i=0;i<7;++i) EXPECT_FLOAT_EQ(h.frames.back().actuators(i).position(),captured.actuators(i).position());
  ASSERT_EQ(h.prepare_command_mode_switch({},KortexHardwareTestAccess::effort()),return_type::OK);
  ASSERT_EQ(h.perform_command_mode_switch({},KortexHardwareTestAccess::effort()),return_type::OK);
  ASSERT_EQ(h.prepare_command_mode_switch(KortexHardwareTestAccess::effort(),{}),return_type::OK);
  ASSERT_EQ(h.perform_command_mode_switch(KortexHardwareTestAccess::effort(),{}),return_type::OK);
  EXPECT_EQ(h.actuator_mode,k_api::ActuatorConfig::TORQUE);
  EXPECT_DOUBLE_EQ(KortexHardwareTestAccess::holdingState(h),0);
}

TEST(EffortInterface, FailedPoweredHoldReportsFailureWithoutEmergencyStop) {
  for(bool fail_mode:{false,true}) {
    SimulatedKortexHardware h;
    KortexHardwareTestAccess::poweredStop(h);
    h.fail_position=fail_mode;h.fail_frame=!fail_mode;
    KortexHardwareTestAccess::requestStop(h,true);
    ASSERT_EQ(writeCommands(h),return_type::OK);
    EXPECT_EQ(h.emergency_stops,0);
    EXPECT_TRUE(KortexHardwareTestAccess::latched(h));
    KortexHardwareTestAccess::requestStop(h,false);
    ASSERT_EQ(writeCommands(h),return_type::OK);
    EXPECT_EQ(h.clear_faults,0);
    EXPECT_EQ(h.emergency_stops,0);
    EXPECT_TRUE(KortexHardwareTestAccess::latched(h));
  }
}

TEST(EffortInterface, HardwareFaultAndWatchdogOverridePoweredHold) {
  for(bool watchdog:{false,true}) {
    SimulatedKortexHardware h;
    KortexHardwareTestAccess::poweredStop(h);
    KortexHardwareTestAccess::requestStop(h,true);
    ASSERT_EQ(writeCommands(h),return_type::OK);
    if(watchdog) KortexHardwareTestAccess::expireWatchdog(h);
    else {KortexHardwareTestAccess::fault(h,1);KortexHardwareTestAccess::observe(h);}
    const auto sent=h.frames.size();
    ASSERT_EQ(writeCommands(h),return_type::OK);
    EXPECT_GT(h.emergency_stops,0);
    EXPECT_TRUE(KortexHardwareTestAccess::latched(h));
    EXPECT_EQ(h.frames.size(),sent);
  }
}

}  // namespace kortex_driver
