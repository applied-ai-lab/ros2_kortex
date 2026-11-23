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
};
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
}
