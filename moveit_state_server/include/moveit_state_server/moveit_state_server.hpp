#ifndef MOVEIT_STATE_SERVER__MOVEIT_STATE_SERVER_HPP_
#define MOVEIT_STATE_SERVER__MOVEIT_STATE_SERVER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <moveit_state_server_msgs/action/go_to_stored_state.hpp>
#include <moveit_state_server_msgs/srv/retrieve_pose.hpp>
#include <moveit_state_server_msgs/srv/store_pose.hpp>

#include <controller_orchestrator/controller_orchestrator.hpp>
#include <moveit_state_server/joint_state_storage.hpp>
namespace moveit_state_server
{

inline void convert( const geometry_msgs::msg::Transform &tf, geometry_msgs::msg::PoseStamped &pose,
                     const std::string &frame )
{
  pose.header.frame_id = frame;
  pose.pose.orientation = tf.rotation;
  pose.pose.position.x = tf.translation.x;
  pose.pose.position.y = tf.translation.y;
  pose.pose.position.z = tf.translation.z;
}

class MoveitStateServer : public rclcpp::Node
{
public:
  explicit MoveitStateServer( const rclcpp::NodeOptions &options = rclcpp::NodeOptions() );
  void initialize();

private:
  /* -------- services -------- */
  void
  storePoseService( const std::shared_ptr<moveit_state_server_msgs::srv::StorePose::Request> request,
                    std::shared_ptr<moveit_state_server_msgs::srv::StorePose::Response> response );

  void retrievePoseService(
      const std::shared_ptr<moveit_state_server_msgs::srv::RetrievePose::Request> request,
      std::shared_ptr<moveit_state_server_msgs::srv::RetrievePose::Response> response );

  /* -------- action callbacks -------- */
  using GoToStateAction = moveit_state_server_msgs::action::GoToStoredState;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GoToStateAction>;

  rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID &,
                                           std::shared_ptr<const GoToStateAction::Goal> goal );

  rclcpp_action::CancelResponse handle_cancel( const std::shared_ptr<GoalHandle> goal_handle );

  void handle_accepted( const std::shared_ptr<GoalHandle> goal_handle );

  /* -------- helpers -------- */
  // bool switchController(bool to_tcp);
  void storeCurrentJointStates( const std::string &name );
  void storeCurrentPose( const std::string &name );
  bool goToStoredJointState( const std::string &name );
  bool goToStoredEndeffectorPosition( const std::string &name );
  void resetJointStateStorage();

  /* -------- parameter callback -------- */
  rcl_interfaces::msg::SetParametersResult
  paramCallback( const std::vector<rclcpp::Parameter> &parameters );

  /* -------- members -------- */
  // MoveIt
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::vector<std::string> joint_names_;
  std::string end_effector_;
  std::string planning_group_{ "arm_group" };

  // Services / actions / subscriptions
  rclcpp::Service<moveit_state_server_msgs::srv::StorePose>::SharedPtr store_pose_service_;
  rclcpp::Service<moveit_state_server_msgs::srv::RetrievePose>::SharedPtr retrieve_pose_service_;
  using ActionServer = rclcpp_action::Server<GoToStateAction>;
  ActionServer::SharedPtr action_server_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::shared_ptr<controller_orchestrator::ControllerOrchestrator> controller_orchestrator_;
  // TF + joint state cache
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  sensor_msgs::msg::JointState last_joint_state_;

  // Persistent storage
  std::unique_ptr<joint_storage::JointStateStorage> joint_state_storage_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> poses_;

  // Parameters
  std::string pose_reference_frame_{ "map" };
  std::string folder_path_;
  std::string robot_name_;
  int planning_attempts_{ 5 };
  double planning_time_{ 2.0 };
  double max_velocity_scaling_factor_{ 0.1 };
  double max_acceleration_scaling_factor_{ 0.1 };

  // Parameter callback handle
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

} // namespace moveit_state_server

#endif // MOVEIT_STATE_SERVER__MOVEIT_STATE_SERVER_HPP_
