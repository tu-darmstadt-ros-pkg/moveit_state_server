#include "moveit_state_server/moveit_state_server.hpp"

#include <moveit/kinematic_constraints/utils.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_state_server/joint_state_file_storage.hpp>

using moveit_state_server_msgs::srv::StorePose;
using moveit_state_server_msgs::srv::RetrievePose;
using moveit_state_server_msgs::action::GoToStoredState;

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace moveit_state_server
{

/* -------------------------------------------------- */
/* Constructor                                        */
/* -------------------------------------------------- */
MoveitStateServer::MoveitStateServer(const rclcpp::NodeOptions &opts)
: rclcpp::Node("moveit_state_server", opts)
{
  /* -------- parameters -------- */
  pose_reference_frame_            = declare_parameter("pose_reference_frame", "world");
  planning_group_                  = declare_parameter("planning_group", "arm_group");
  planning_attempts_               = declare_parameter("planning_attempts", 5);
  planning_time_                   = declare_parameter("planning_time", 2.0);
  max_velocity_scaling_factor_     = declare_parameter("max_velocity_scaling_factor", 0.1);
  max_acceleration_scaling_factor_ = declare_parameter("max_acceleration_scaling_factor", 0.1);
  folder_path_                     = declare_parameter<std::string>("folder_path", "/tmp/moveit_state_server");
  robot_name_                      = declare_parameter<std::string>("robot_name", "");

  /* -------- TF + joint‑state subscription -------- */
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { last_joint_state_ = *msg; });

  /* -------- persistent storage -------- */
  resetJointStateStorage();

  /* -------- services -------- */
  store_pose_service_ = create_service<StorePose>(
      "store_arm_pose",
      std::bind(&MoveitStateServer::storePoseService, this, _1, _2));

  retrieve_pose_service_ = create_service<RetrievePose>(
      "retrieve_arm_pose",
      std::bind(&MoveitStateServer::retrievePoseService, this, _1, _2));


  /* -------- param callback -------- */
  param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&MoveitStateServer::paramCallback, this, _1));

  RCLCPP_INFO(get_logger(), "MoveitStateServer ready (group: %s, eef: %s)",
              planning_group_.c_str(), end_effector_.c_str());
}

void MoveitStateServer::initialize() {
  moveit::planning_interface::MoveGroupInterface::Options options(planning_group_);
  options.move_group_namespace = get_namespace();
  RCLCPP_WARN( get_logger(), "Move group namespace: %s", options.move_group_namespace.c_str() );

  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                     shared_from_this(), options);
  RCLCPP_INFO( get_logger(), "Move group '%s' initialized", planning_group_.c_str() );
  move_group_->setPlanningTime(planning_time_);
  move_group_->setNumPlanningAttempts(planning_attempts_);
  move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
  move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);
  joint_names_  = move_group_->getJointNames();
  end_effector_ = move_group_->getEndEffectorLink();
        RCLCPP_INFO(get_logger(), "Move group '%s' has %zu joints, end effector: %s",
                planning_group_.c_str(), joint_names_.size(), end_effector_.c_str());
  action_server_ = rclcpp_action::create_server<GoToStoredState>(
     shared_from_this(),
     "move_arm_to_stored_pose",
     std::bind(&MoveitStateServer::handle_goal,     this, _1, _2),
     std::bind(&MoveitStateServer::handle_cancel,   this, _1),
     std::bind(&MoveitStateServer::handle_accepted, this, _1));
  controller_orchestrator_ =
      std::make_shared<controller_orchestrator::ControllerOrchestrator>(shared_from_this());
}

/* -------------------------------------------------- */
/* Services                                           */
/* -------------------------------------------------- */
void MoveitStateServer::storePoseService(
    const std::shared_ptr<StorePose::Request>  req,
    std::shared_ptr<StorePose::Response>       res)
{
  RCLCPP_INFO( get_logger(), "Received Request to store pose" );
  if (req->mode == StorePose::Request::STORE_JOINT_POSITIONS) // TODO store values with group information
    storeCurrentJointStates(req->name);
  else
    storeCurrentPose(req->name);

  res->success.data = true;
}

void MoveitStateServer::retrievePoseService(
    const std::shared_ptr<RetrievePose::Request>  req,
    std::shared_ptr<RetrievePose::Response>       res)
{

  if (req->mode == RetrievePose::Request::RETRIEVE_END_EFFECTOR_POSE) {
    auto it = poses_.find(req->name);
    if (it != poses_.end()) res->pose = it->second;
  } else {
    bool ok = joint_state_storage_->getStoredJointState(req->name, res->joint_state, false);
    if (!ok) RCLCPP_WARN(get_logger(), "Joint state '%s' not found", req->name.c_str());
  }
}

/* -------------------------------------------------- */
/* Joint / pose storage helpers                       */
/* -------------------------------------------------- */
void MoveitStateServer::storeCurrentJointStates(const std::string &name)
{
  if (last_joint_state_.name.empty()) {
    RCLCPP_WARN(get_logger(), "No joint_states received yet – cannot store");
    return;
  }
  joint_state_storage_->addJointState(last_joint_state_, name);
}

void MoveitStateServer::storeCurrentPose(const std::string &name)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(pose_reference_frame_, end_effector_, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  convert(tf.transform, pose, pose_reference_frame_);
  poses_[name] = pose;
}

/* -------------------------------------------------- */
/* Action server                                      */
/* -------------------------------------------------- */
rclcpp_action::GoalResponse MoveitStateServer::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GoToStoredState::Goal> goal)
{
  bool ok = (goal->mode == GoToStoredState::Goal::GO_TO_STORED_JOINT_POSITIONS)
              ? joint_state_storage_->isJointStateStored(goal->name, true)
              : poses_.count(goal->name) > 0;

  return ok ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
            : rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse MoveitStateServer::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
{
  (void)goal_handle;
  move_group_->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveitStateServer::handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{[this, goal_handle]()
  {
    const auto goal = goal_handle->get_goal();
    bool success{false};
    std::vector<std::string> activate_controllers = {"arm_trajectory_controller",
                                                     "gripper_trajectory_controller"};
    controller_orchestrator_->smartSwitchController( activate_controllers );

    if (goal->mode == GoToStoredState::Goal::GO_TO_STORED_JOINT_POSITIONS)
      success = goToStoredJointState(goal->name);
    else
      success = goToStoredEndeffectorPosition(goal->name);

    //switchController(true);

    auto result = std::make_shared<GoToStateAction::Result>();
    if (success)
      goal_handle->succeed(result);
    else
      goal_handle->abort(result);
  }}.detach();
}

/* -------------------------------------------------- */
/* Move‑group helpers                                 */
/* -------------------------------------------------- */
bool MoveitStateServer::goToStoredJointState(const std::string &name)
{
  sensor_msgs::msg::JointState js;
  if (!joint_state_storage_->getStoredJointState(name, js, false))
    return false;
  std::stringstream ss;
  ss<<"Moving to stored joint state: " << name<<"\n ";
  for (size_t i = 0; i < js.position.size(); i++) ss << js.name[i] << ": " << js.position[i] << "\n ";
  RCLCPP_INFO( get_logger(), "%s", ss.str().c_str() );
  moveit::core::RobotState goal_state(*move_group_->getCurrentState());
  goal_state.setVariablePositions(js.name, js.position);
  move_group_->setJointValueTarget(goal_state);
  move_group_->setStartStateToCurrentState();
  return move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
}

bool MoveitStateServer::goToStoredEndeffectorPosition(const std::string &name)
{
  move_group_->setPoseTarget(poses_.at(name), end_effector_);
  return move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
}

/* -------------------------------------------------- */
/* Joint‑state storage backend                        */
/* -------------------------------------------------- */
void MoveitStateServer::resetJointStateStorage()
{
  joint_state_storage_ = std::make_unique<joint_storage::JointStateFileStorage>(folder_path_, robot_name_);
  joint_state_storage_->loadAllJointStates();
}

/* -------------------------------------------------- */
/* Parameter callback                                 */
/* -------------------------------------------------- */
rcl_interfaces::msg::SetParametersResult
MoveitStateServer::paramCallback(const std::vector<rclcpp::Parameter> &params)
{
  for (const auto &p : params) {
    if (p.get_name() == "planning_time") {
      planning_time_ = p.as_double();
      move_group_->setPlanningTime(planning_time_);
    } else if (p.get_name() == "max_velocity_scaling_factor") {
      max_velocity_scaling_factor_ = p.as_double();
      move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
    } else if (p.get_name() == "max_acceleration_scaling_factor") {
      max_acceleration_scaling_factor_ = p.as_double();
      move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);
    }
  }
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  return res;
}

}  // namespace moveit_state_server

/* -------------------------------------------------- */
/* main                                               */
/* -------------------------------------------------- */
int main(int argc, char **argv )
{
  rclcpp::init( argc, argv );
  const auto node = std::make_shared<moveit_state_server::MoveitStateServer>();
  node->initialize();
  const auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
