#ifndef MOVEIT_STATE_SERVER__GOAL_EXECUTION_HPP_
#define MOVEIT_STATE_SERVER__GOAL_EXECUTION_HPP_

#include <string>
#include <vector>

namespace moveit_state_server
{

/**
 * The controllers a motion needs: the arm's, plus the gripper's only when a gripper joint moves,
 * that is one in both @p group_joints and @p target_joints. Otherwise the gripper's controller is
 * left alone, so a grasp it holds is kept.
 */
std::vector<std::string> controllersForMotion( const std::vector<std::string> &group_joints,
                                               const std::vector<std::string> &target_joints );

} // namespace moveit_state_server

#endif // MOVEIT_STATE_SERVER__GOAL_EXECUTION_HPP_
