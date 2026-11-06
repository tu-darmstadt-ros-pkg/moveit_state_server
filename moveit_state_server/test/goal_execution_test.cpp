#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <moveit_state_server/goal_execution.hpp>

using moveit_state_server::controllersForMotion;
using Names = std::vector<std::string>;

TEST( ControllersForMotion, AnArmMotionLeavesTheGripperControllerAlone )
{
  const Names arm = { "arm_joint_1", "arm_joint_2" };
  // A stored joint state holds every joint of /joint_states, the gripper's too
  EXPECT_EQ( controllersForMotion( arm, { "arm_joint_1", "arm_joint_2", "gripper_servo_joint" } ),
             Names{ "arm_trajectory_controller" } );
}

TEST( ControllersForMotion, AGripperJointInTheMotionNeedsTheGripperController )
{
  const Names arm_with_gripper = { "arm_joint_1", "gripper_servo_joint" };
  EXPECT_EQ( controllersForMotion( arm_with_gripper, { "arm_joint_1", "gripper_servo_joint" } ),
             ( Names{ "arm_trajectory_controller", "gripper_trajectory_controller" } ) );
  EXPECT_EQ( controllersForMotion( arm_with_gripper, { "arm_joint_1" } ),
             Names{ "arm_trajectory_controller" } );
}
