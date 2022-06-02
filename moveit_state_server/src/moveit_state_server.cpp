#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_state_server/moveit_state_server.h>
#include <controller_manager_msgs/SwitchController.h>
// controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

namespace moveit_state_server
{
MoveitStateServer::MoveitStateServer() : as_(nh_, "/move_arm_to_stored_pose", false)
{
  pnh_ = ros::NodeHandle("~");
  if (!ros::service::waitForService("/get_planning_scene", ros::Duration(10.0)))
  {
    ROS_WARN_STREAM("[moveit_state_server] Moveit is not available on the robot. MoveitStateServer won't work.");
  }
  else
  {
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description", false);
    moveit_robot_model_ = robot_model_loader.getModel();
    store_pose_service = pnh_.advertiseService("/store_arm_joint_states", &MoveitStateServer::storePoseService, this);
    retrieve_pose_server =
        pnh_.advertiseService("/retrieve_arm_joint_states", &MoveitStateServer::retrievePoseService, this);
    get_planning_scene_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    switch_controllers_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("/manipulator_arm_control/"
                                                                                       "controller_manager/"
                                                                                       "switch_controller");
    move_ac_ = std::make_unique<actionlib::ActionClient<moveit_msgs::MoveGroupAction>>("move_group");
    moveit_status_sub_ = nh_.subscribe("move_group/feedback", 10, &MoveitStateServer::moveitStatusCB, this);
    try
    {
      planning_group_ = "arm_group";
      move_group_interface_.reset(new moveit::planning_interface::MoveGroupInterface(planning_group_));
      const moveit::core::JointModelGroup *joint_model_group =
          move_group_interface_->getRobotModel()->getJointModelGroup(planning_group_);
      joint_names_ = joint_model_group->getActiveJointModelNames();
    }
    catch (...)
    {
      ROS_WARN_STREAM("[moveit_state_server] Planning group "
                      << planning_group_ << " seems to be unavailable. MoveitStateServer won't work.");
    }

    as_.registerGoalCallback([this] { goalCB(); });
    as_.registerPreemptCallback([this] { preemptCB(); });
    as_.start();
  }
}
void MoveitStateServer::storeCurrentJointStates(std::vector<double> &joint_states)
{
  planning_scene::PlanningScene planning_scene(moveit_robot_model_);
  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components = srv.request.components.ROBOT_STATE;
  joint_states.resize(joint_names_.size());
  if (this->get_planning_scene_.call(srv))
  {
    for (int i = 0; i < srv.response.scene.robot_state.joint_state.name.size(); i++)
    {
      for (int j = 0; j < joint_names_.size(); j++)
      {
        if (srv.response.scene.robot_state.joint_state.name[i] == joint_names_[j])
        {
          joint_states[j] = srv.response.scene.robot_state.joint_state.position[i];
        }
      }
      ROS_INFO_STREAM(srv.response.scene.robot_state.joint_state.name[i]
                      << " " << srv.response.scene.robot_state.joint_state.position[i]);
    }
  }
}
bool MoveitStateServer::storePoseService(moveit_state_server_msgs::StorePoseRequest &req,
                                         moveit_state_server_msgs::StorePoseResponse &res)
{
  storeCurrentJointStates(joint_states_);
  res.success.data = true;
  return true;
}
bool MoveitStateServer::retrievePoseService(moveit_state_server_msgs::RetrievePoseRequest &req,
                                            moveit_state_server_msgs::RetrievePoseResponse &res)
{
  res.joint_states = joint_states_;
  res.names = joint_names_;
  return true;
}
bool MoveitStateServer::switchController(bool to_tcp)
{
  controller_manager_msgs::SwitchController switchController;
  std::vector<std::string> start_controllers = { "arm_tcp_controller" };
  std::vector<std::string> stop_controllers = { "manipulator_arm_traj_controller", "gripper_traj_controller" };
  if (to_tcp)
  {
    switchController.request.start_controllers = start_controllers;
    switchController.request.stop_controllers = stop_controllers;
  }
  else
  {
    switchController.request.start_controllers = stop_controllers;
    switchController.request.stop_controllers = start_controllers;
  }
  switchController.request.strictness = 1;
  return this->switch_controllers_.call(switchController);
}
void MoveitStateServer::goalCB()
{
  std::vector<double> current_joint_states;
  storeCurrentJointStates(current_joint_states);
  if (switchController(false))
  {
    moveit_msgs::MoveGroupGoal msg;
    msg.request.allowed_planning_time = 2;
    msg.request.group_name = "arm_group";
    msg.request.start_state.joint_state.name = joint_names_;
    msg.request.start_state.joint_state.position = current_joint_states;
    msg.request.max_acceleration_scaling_factor = 0.5;
    moveit_msgs::Constraints goal_constraints;
    for (int i = 0; i < joint_names_.size(); i++)
    {
      moveit_msgs::JointConstraint jointConstraint;
      jointConstraint.joint_name = joint_names_[i];
      jointConstraint.position = joint_states_[i];
      goal_constraints.joint_constraints.push_back(jointConstraint);
    }
    msg.request.goal_constraints.push_back(goal_constraints);
    started_arm_movement_ = true;
    move_ac_->sendGoal(msg);
  }
}

void MoveitStateServer::preemptCB()
{
  ROS_INFO_STREAM("[moveit_state_server] Preempted moveit_state_server.");
}
void MoveitStateServer::moveitStatusCB(const moveit_msgs::MoveGroupActionFeedback &msg)
{
  //wait until the arm movement finishes or is canceled
  if (started_arm_movement_ && (msg.feedback.state != "PLANNING" && msg.feedback.state != "MONITOR"))
  {
    switchController(true);
    started_arm_movement_ = false;
  }
}

}  // namespace moveit_state_server
int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  moveit_state_server::MoveitStateServer stateServer;

  ros::spin();

  return 0;
}
