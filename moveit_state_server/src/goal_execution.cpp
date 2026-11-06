#include "moveit_state_server/goal_execution.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <utility>

namespace moveit_state_server
{

namespace
{
const std::vector<std::string> kGripperJoints = { "gripper_servo_joint" };

bool contains( const std::vector<std::string> &names, const std::string &name )
{
  return std::find( names.begin(), names.end(), name ) != names.end();
}
} // namespace

std::vector<std::string> controllersForMotion( const std::vector<std::string> &group_joints,
                                               const std::vector<std::string> &target_joints )
{
  std::vector<std::string> controllers = { "arm_trajectory_controller" };
  const bool moves_gripper =
      std::any_of( kGripperJoints.begin(), kGripperJoints.end(), [&]( const std::string &joint ) {
        return contains( group_joints, joint ) && contains( target_joints, joint );
      } );
  if ( moves_gripper )
    controllers.push_back( "gripper_trajectory_controller" );
  return controllers;
}

std::string switchAndMove( const controller_orchestrator::ControllerOrchestrator &orchestrator,
                           const std::vector<std::string> &controllers,
                           const std::function<bool()> &move, int timeout_s )
{
  // The async call reports the controller manager's reason, which the blocking one only logs
  auto answer = std::make_shared<std::promise<std::pair<bool, std::string>>>();
  auto answered = answer->get_future();
  orchestrator.smartSwitchControllerAsync(
      controllers,
      [answer]( bool success, const std::string &message ) {
        answer->set_value( { success, message } );
      },
      timeout_s );
  // The orchestrator answers within its timeout plus one second, unless it is destroyed first
  const auto bound = std::chrono::seconds( timeout_s + 2 );
  if ( answered.wait_for( bound ) != std::future_status::ready )
    return "Controller switch failed: no answer within " + std::to_string( bound.count() ) + " s";
  const auto [switched, reason] = answered.get();
  if ( !switched )
    return "Controller switch failed: " + reason;
  return move() ? "" : "Motion failed";
}

moveit_state_server_msgs::action::GoToStoredState::Result goalResult( const std::string &failure )
{
  moveit_state_server_msgs::action::GoToStoredState::Result result;
  result.success = failure.empty();
  result.message = failure;
  return result;
}

} // namespace moveit_state_server
