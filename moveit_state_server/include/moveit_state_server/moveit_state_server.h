#ifndef MOVEIT_STATE_SERVER_H
#define MOVEIT_STATE_SERVER_H


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_state_server_msgs/StorePose.h>
#include <moveit_state_server_msgs/RetrievePose.h>
#include <moveit_state_server_msgs/GoToStoredStateAction.h>

namespace moveit_state_server
{

class MoveitStateServer
{
public:
  MoveitStateServer();
private:
  bool storePoseService( moveit_state_server_msgs::StorePoseRequest &req,
                         moveit_state_server_msgs::StorePoseResponse &res );
  bool retrievePoseService( moveit_state_server_msgs::RetrievePoseRequest &req,
                         moveit_state_server_msgs::RetrievePoseResponse &res );
  void goalCB();
  void preemptCB();
  bool switchController(bool to_tcp);
  void moveitStatusCB(const moveit_msgs::MoveGroupActionFeedback& msg);
  void storeCurrentJointStates(std::vector<double> &joint_states);
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string planning_group_ = "arm_group";
  ros::ServiceServer store_pose_service;
  ros::ServiceServer retrieve_pose_server;
  robot_model::RobotModelPtr moveit_robot_model_;
  actionlib::SimpleActionServer<moveit_state_server_msgs::GoToStoredStateAction> as_;
  std::unique_ptr<actionlib::ActionClient<moveit_msgs::MoveGroupAction>> move_ac_;
  ros::Subscriber moveit_status_sub_;
  ros::ServiceClient get_planning_scene_;
  ros::ServiceClient switch_controllers_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_interface_;
  std::vector<double> joint_states_;
  std::vector<std::string> joint_names_;
  bool started_arm_movement_=false;
private:
};
}
#endif //MOVEIT_STATE_SERVER_H