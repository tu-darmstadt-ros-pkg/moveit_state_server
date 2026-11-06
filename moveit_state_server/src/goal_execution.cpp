#include "moveit_state_server/goal_execution.hpp"

#include <algorithm>

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

} // namespace moveit_state_server
