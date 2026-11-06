#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <moveit_state_server/goal_execution.hpp>
#include <rclcpp/rclcpp.hpp>

using controller_manager_msgs::srv::SwitchController;
using moveit_state_server::controllersForMotion;
using moveit_state_server::switchAndMove;
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

// A controller manager that answers every switch with a fixed verdict
class SwitchAndMoveTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>( "switch_and_move_test" );
    service_ = node_->create_service<SwitchController>(
        "controller_manager/switch_controller",
        [this]( const std::shared_ptr<SwitchController::Request>,
                const std::shared_ptr<SwitchController::Response> response ) {
          response->ok = accept_;
          response->message = accept_ ? "" : "e-stop engaged";
        } );
    executor_.add_node( node_ );
    spinner_ = std::thread( [this] { executor_.spin(); } );
    orchestrator_ = std::make_unique<controller_orchestrator::ControllerOrchestrator>( node_ );
    const auto probe =
        node_->create_client<SwitchController>( "controller_manager/switch_controller" );
    ASSERT_TRUE( probe->wait_for_service( std::chrono::seconds( 5 ) ) );
  }

  void TearDown() override
  {
    orchestrator_.reset();
    executor_.cancel();
    spinner_.join();
  }

  std::atomic<bool> accept_{ true };
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<SwitchController>::SharedPtr service_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spinner_;
  std::unique_ptr<controller_orchestrator::ControllerOrchestrator> orchestrator_;
};

TEST_F( SwitchAndMoveTest, ARefusedSwitchAbortsWithTheReasonAndDoesNotMove )
{
  accept_ = false;
  bool moved = false;
  const std::string message = switchAndMove( *orchestrator_, { "arm_trajectory_controller" }, [&] {
    moved = true;
    return true;
  } );
  EXPECT_FALSE( moved );
  EXPECT_EQ( message, "Controller switch failed: e-stop engaged" );
}

TEST_F( SwitchAndMoveTest, AnAcceptedSwitchMoves )
{
  bool moved = false;
  EXPECT_EQ( switchAndMove( *orchestrator_, { "arm_trajectory_controller" },
                            [&] {
                              moved = true;
                              return true;
                            } ),
             "" );
  EXPECT_TRUE( moved );
  EXPECT_EQ( switchAndMove( *orchestrator_, { "arm_trajectory_controller" }, [] { return false; } ),
             "Motion failed" );
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
