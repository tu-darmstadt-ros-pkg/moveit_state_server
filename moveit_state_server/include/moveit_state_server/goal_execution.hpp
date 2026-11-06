#ifndef MOVEIT_STATE_SERVER__GOAL_EXECUTION_HPP_
#define MOVEIT_STATE_SERVER__GOAL_EXECUTION_HPP_

#include <functional>
#include <string>
#include <vector>

#include <controller_orchestrator/controller_orchestrator.hpp>
#include <moveit_state_server_msgs/action/go_to_stored_state.hpp>

namespace moveit_state_server
{

/**
 * The controllers a motion needs: the arm's, plus the gripper's only when a gripper joint moves,
 * that is one in both @p group_joints and @p target_joints. Otherwise the gripper's controller is
 * left alone, so a grasp it holds is kept.
 */
std::vector<std::string> controllersForMotion( const std::vector<std::string> &group_joints,
                                               const std::vector<std::string> &target_joints );

/**
 * Switches to @p controllers within @p timeout_s, then runs @p move unless the switch failed.
 * Waits at most two seconds longer for the switch's answer.
 * @return Empty on success, otherwise why the goal failed.
 */
std::string switchAndMove( const controller_orchestrator::ControllerOrchestrator &orchestrator,
                           const std::vector<std::string> &controllers,
                           const std::function<bool()> &move, int timeout_s = 5 );

/// The goal's result for what switchAndMove returned.
moveit_state_server_msgs::action::GoToStoredState::Result goalResult( const std::string &failure );

} // namespace moveit_state_server

#endif // MOVEIT_STATE_SERVER__GOAL_EXECUTION_HPP_
