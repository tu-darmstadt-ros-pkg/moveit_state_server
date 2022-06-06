#ifndef MOVEIT_STATE_SERVER_H
#define MOVEIT_STATE_SERVER_H

#include <actionlib/server/simple_action_server.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_state_server_msgs/GoToStoredStateAction.h>
#include <moveit_state_server_msgs/RetrievePose.h>
#include <moveit_state_server_msgs/StorePose.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

namespace moveit_state_server
{
inline
    void convert(const geometry_msgs::Transform& trans, geometry_msgs::Pose& pose)
{
  pose.orientation = trans.rotation;
  pose.position.x = trans.translation.x;
  pose.position.y = trans.translation.y;
  pose.position.z = trans.translation.z;
}

class MoveitStateServer
{
public:
  MoveitStateServer();
private:
  bool storePoseService( moveit_state_server_msgs::StorePoseRequest &req,
                         moveit_state_server_msgs::StorePoseResponse &res );
  bool retrievePoseService( moveit_state_server_msgs::RetrievePoseRequest &req,
                         moveit_state_server_msgs::RetrievePoseResponse &res );
  void goalCB(const moveit_state_server_msgs::GoToStoredStateGoalConstPtr &goal);
  void preemptCB();
  bool switchController(bool to_tcp);
  void storeCurrentJointStates(std::vector<double> &joint_states);
  void storeCurrentPose();
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string planning_group_ = "arm_group";
  ros::ServiceServer store_pose_service;
  ros::ServiceServer retrieve_pose_server;
  robot_model::RobotModelPtr moveit_robot_model_;
  actionlib::SimpleActionServer<moveit_state_server_msgs::GoToStoredStateAction> as_;
  std::unique_ptr<actionlib::ActionClient<moveit_msgs::MoveGroupAction>> move_ac_;
  std::unique_ptr<tf2_ros::Buffer>buffer_;
  std::unique_ptr<tf2_ros::TransformListener>listener_;
  ros::ServiceClient get_planning_scene_;
  ros::ServiceClient switch_controllers_;
  std::vector<double> joint_states_;
  std::string end_effector_;
  std::string store_pose_service_name_;
  std::vector<std::string> joint_names_;
  bool stored_joint_positions_=false;
  bool stored_end_effector_position_=false;
  geometry_msgs::Pose pose_;
private:
};
}
#endif //MOVEIT_STATE_SERVER_H