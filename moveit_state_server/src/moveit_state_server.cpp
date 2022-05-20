#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_state_server/moveit_state_server.h>
#include <controller_manager_msgs/SwitchController.h>
// controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

namespace moveit_state_server
{
MoveitStateServer::MoveitStateServer():as_(nh_, "/move_arm_to_stored_pose", false)
{
  pnh_ = ros::NodeHandle( "~" );
  ros::Duration(5).sleep();
  if(!ros::service::exists("/get_planning_scene",false)){
        ROS_WARN_STREAM("[moveit_state_server] Moveit is not available on the robot. MoveitStateServer won't work.");
  }else{
    robot_model_loader::RobotModelLoader robot_model_loader( "robot_description", false );
    moveit_robot_model_ = robot_model_loader.getModel();
    store_pose_service =
        pnh_.advertiseService( "/store_arm_joint_states", &MoveitStateServer::storePoseService, this );
    retrieve_pose_server =
        pnh_.advertiseService( "/retrieve_arm_joint_states", &MoveitStateServer::retrievePoseService, this );
    get_planning_scene_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    switch_controllers_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("/manipulator_arm_control/controller_manager/switch_controller");
    try{
      planning_group_ = "arm_group";
      move_group_interface_.reset( new moveit::planning_interface::MoveGroupInterface( planning_group_ ) );
      const moveit::core::JointModelGroup *joint_model_group =
          move_group_interface_->getRobotModel()->getJointModelGroup( planning_group_ );
      joint_names_ =joint_model_group->getActiveJointModelNames();
    }catch(...){
      ROS_WARN_STREAM("[moveit_state_server] Planning group "<<planning_group_<<" seems to be unavailable. MoveitStateServer won't work.");
    }


    as_.registerGoalCallback([this] { goalCB(); });
    as_.registerPreemptCallback([this] { preemptCB(); });
    as_.start();
  }
}

bool MoveitStateServer::storePoseService( moveit_state_server_msgs::StorePoseRequest &req,
                                          moveit_state_server_msgs::StorePoseResponse &res )
{

  planning_scene::PlanningScene planning_scene( moveit_robot_model_ );
  // ignore collision between these links, only padding collides and because of this camera cannot look upwards
  planning_scene.getAllowedCollisionMatrixNonConst().setEntry( "sensor_head_thermal_cam_frame",
                                                               "chassis_link", true );
  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components = srv.request.components.ROBOT_STATE;
  joint_states_.resize(joint_names_.size());
  if ( this->get_planning_scene_.call( srv ) ) {
    for ( int i = 0; i < srv.response.scene.robot_state.joint_state.name.size(); i++ ) {
      for(int j=0;j<joint_names_.size();j++){
        if(srv.response.scene.robot_state.joint_state.name[i]==joint_names_[j]){
          joint_states_[j] = srv.response.scene.robot_state.joint_state.position[i];
        }
      }
      ROS_INFO_STREAM(
          srv.response.scene.robot_state.joint_state.name[i]<<" "<<
          srv.response.scene.robot_state.joint_state.position[i] );
    }
  }
  res.success.data = true;
  return true;
}
bool MoveitStateServer::retrievePoseService( moveit_state_server_msgs::RetrievePoseRequest &req,
                          moveit_state_server_msgs::RetrievePoseResponse &res ){
  res.joint_states = joint_states_;
  res.names = joint_names_;
  return true;
}
void MoveitStateServer::goalCB() {
  controller_manager_msgs::SwitchController switchController;
  std::vector<std::string> start_controllers = {"arm_tcp_controller"};
  std::vector<std::string> stop_controllers = {"manipulator_arm_traj_controller","gripper_traj_controller"};
  switchController.request.start_controllers = start_controllers;
  switchController.request.stop_controllers = stop_controllers;
  switchController.request.strictness = 1;
  // get current state TODO combine with  function above
  planning_scene::PlanningScene planning_scene( moveit_robot_model_ );
 std::vector<double> current_joint_states;
  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components = srv.request.components.ROBOT_STATE;
  current_joint_states.resize(joint_names_.size());
  if ( this->get_planning_scene_.call( srv ) ) {
    for ( int i = 0; i < srv.response.scene.robot_state.joint_state.name.size(); i++ ) {
      for(int j=0;j<joint_names_.size();j++){
        if(srv.response.scene.robot_state.joint_state.name[i]==joint_names_[j]){
          current_joint_states[j] = srv.response.scene.robot_state.joint_state.position[i];
        }
      }
      ROS_INFO_STREAM(
          srv.response.scene.robot_state.joint_state.name[i]<<" "<<
          srv.response.scene.robot_state.joint_state.position[i] );
    }
  }
  if ( this->switch_controllers_.call( switchController ) ) {
    actionlib::ActionClient<moveit_msgs::MoveGroupAction> move_ac("move_group");
    moveit_msgs::MoveGroupGoal msg;
    msg.request.allowed_planning_time = 2;
    msg.request.group_name = "arm_group";
    msg.request.start_state.joint_state.name = joint_names_;
    msg.request.start_state.joint_state.position = current_joint_states;
    moveit_msgs::Constraints goal_constraints;
    for(int i=0;i<joint_names_.size();i++){
      moveit_msgs::JointConstraint jointConstraint;
      jointConstraint.joint_name = joint_names_[i];
      jointConstraint.position = joint_states_[i];
      goal_constraints.joint_constraints.push_back(jointConstraint);
    }
    msg.request.goal_constraints.push_back(goal_constraints);
    move_ac.sendGoal(msg);

    switchController.request.start_controllers = stop_controllers;
    switchController.request.stop_controllers = start_controllers;
    this->switch_controllers_.call( switchController );
  }

}
void MoveitStateServer::preemptCB(){
  ROS_INFO_STREAM("[moveit_state_server] Preempted moveit_state_server.");
}

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  moveit_state_server::MoveitStateServer stateServer;

  ros::spin();

  return 0;
}


