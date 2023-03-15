#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_state_server/moveit_state_server.h>
#include <controller_manager_msgs/SwitchController.h>
#include <moveit/kinematic_constraints/utils.h>
// controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

namespace moveit_state_server {
    MoveitStateServer::MoveitStateServer() : as_(nh_, "/move_arm_to_stored_pose",
                                                 [this](auto &&PH1) { goalCB(std::forward<decltype(PH1)>(PH1)); },
                                                 false) {
        pnh_ = ros::NodeHandle("~");
        if (!ros::service::waitForService("/get_planning_scene", ros::Duration(10.0))) {
            ROS_WARN_STREAM(
                    "[moveit_state_server] Moveit is not available on the robot. MoveitStateServer won't work.");
        } else {
            robot_model_loader::RobotModelLoader robot_model_loader("robot_description", false);
            moveit_robot_model_ = robot_model_loader.getModel();
            store_pose_service_name_ = "/store_arm_joint_states";
            store_pose_service = pnh_.advertiseService(store_pose_service_name_, &MoveitStateServer::storePoseService,
                                                       this);
            retrieve_pose_server =
                    pnh_.advertiseService("/retrieve_arm_joint_states", &MoveitStateServer::retrievePoseService, this);
            get_planning_scene_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
            switch_controllers_ = nh_.serviceClient<controller_manager_msgs::SwitchController>(
                    "/manipulator_arm_control/"
                    "controller_manager/"
                    "switch_controller");
            move_ac_ = std::make_unique<actionlib::ActionClient<moveit_msgs::MoveGroupAction>>("move_group");
            try {
                planning_group_ = "arm_group";
                moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(moveit_robot_model_));
                kinematic_state->setToDefaultValues();
                const moveit::core::JointModelGroup *joint_model_group = moveit_robot_model_->getJointModelGroup(
                        planning_group_);
                joint_names_ = joint_model_group->getActiveJointModelNames();
                end_effector_ = joint_model_group->getLinkModelNames().back();
                ROS_WARN_STREAM("###" << end_effector_);
            }
            catch (...) {
                ROS_WARN_STREAM("[moveit_state_server] Planning group "
                                        << planning_group_
                                        << " seems to be unavailable. MoveitStateServer won't work.");
            }
            buffer_ = std::make_unique<tf2_ros::Buffer>();
            listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
            as_.registerPreemptCallback([this] { preemptCB(); });
            as_.start();
        }
    }

    void MoveitStateServer::storeCurrentJointStates(std::vector<double> &joint_states) {
        moveit_msgs::GetPlanningScene srv;
        srv.request.components.components = srv.request.components.ROBOT_STATE;
        joint_states.resize(joint_names_.size());
        if (this->get_planning_scene_.call(srv)) {
            for (int i = 0; i < srv.response.scene.robot_state.joint_state.name.size(); i++) {
                for (int j = 0; j < joint_names_.size(); j++) {
                    if (srv.response.scene.robot_state.joint_state.name[i] == joint_names_[j]) {
                        joint_states[j] = srv.response.scene.robot_state.joint_state.position[i];
                    }
                }
            }
        }
    }

    void MoveitStateServer::storeCurrentPose() {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = buffer_->lookupTransform("world", end_effector_, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
        convert(transformStamped.transform, pose_);
    }

    bool MoveitStateServer::storePoseService(moveit_state_server_msgs::StorePoseRequest &req,
                                             moveit_state_server_msgs::StorePoseResponse &res) {
        if (req.mode == moveit_state_server_msgs::StorePoseRequest::STORE_JOINT_POSITIONS) {
            storeCurrentJointStates(joint_states_);
            stored_joint_positions_ = true;
        }

        if (req.mode == moveit_state_server_msgs::StorePoseRequest::STORE_END_EFFECTOR_POSE) {
            storeCurrentPose();
            stored_end_effector_position_ = true;
        }

        res.success.data = true;
        return true;
    }

    bool MoveitStateServer::retrievePoseService(moveit_state_server_msgs::RetrievePoseRequest &req,
                                                moveit_state_server_msgs::RetrievePoseResponse &res) {
        res.joint_states = joint_states_;
        res.names = joint_names_;
        return true;
    }

    bool MoveitStateServer::switchController(bool to_tcp) {
        controller_manager_msgs::SwitchController switchController;
        std::vector<std::string> start_controllers = {"arm_tcp_controller"};
        std::vector<std::string> stop_controllers = {"manipulator_arm_traj_controller", "gripper_traj_controller"};
        if (to_tcp) {
            switchController.request.start_controllers = start_controllers;
            switchController.request.stop_controllers = stop_controllers;
        } else {
            switchController.request.start_controllers = stop_controllers;
            switchController.request.stop_controllers = start_controllers;
        }
        switchController.request.strictness = 1;
        return this->switch_controllers_.call(switchController);
    }

    void MoveitStateServer::goalCB(const moveit_state_server_msgs::GoToStoredStateGoalConstPtr &goal) {
        //verify that a pose has been previously stored
        if (goal->mode == moveit_state_server_msgs::GoToStoredStateGoal::GO_TO_STORED_JOINT_POSITIONS and
            !stored_joint_positions_) {
            ROS_WARN_STREAM(
                    "[moveit_state_server] Before moving the arm to stored joint states, joint positions must be stored by calling the service "
                            << store_pose_service_name_);
            as_.setAborted();
            return;
        }
        if (goal->mode == moveit_state_server_msgs::GoToStoredStateGoal::GO_TO_STORED_END_EFFECTOR_POSE and
            !stored_end_effector_position_) {
            ROS_WARN_STREAM(
                    "[moveit_state_server] Before moving the arm to stored a stored eef pose, the pose must be stored by calling the service "
                            << store_pose_service_name_);
            as_.setAborted();
            return;
        }
        std::vector<double> current_joint_states;
        storeCurrentJointStates(current_joint_states);
        if (switchController(false)) {
            moveit_msgs::MoveGroupGoal msg;
            msg.planning_options.plan_only = false; // plan and move arrm
            msg.planning_options.look_around = false;
            msg.planning_options.replan = false;
            msg.planning_options.planning_scene_diff.is_diff = true;
            msg.planning_options.planning_scene_diff.robot_state.is_diff = true;
            msg.request.start_state.is_diff = true;
            msg.request.allowed_planning_time = 2;
            msg.request.num_planning_attempts = 3;
            msg.request.group_name = "arm_group";
            msg.request.start_state.joint_state.name = joint_names_;
            msg.request.start_state.joint_state.position = current_joint_states;
            msg.request.max_acceleration_scaling_factor = 0.5;
            if (goal->mode == moveit_state_server_msgs::GoToStoredStateGoal::GO_TO_STORED_JOINT_POSITIONS) {
                moveit_msgs::Constraints goal_constraints;
                for (int i = 0; i < joint_names_.size(); i++) {
                    moveit_msgs::JointConstraint jointConstraint;
                    jointConstraint.joint_name = joint_names_[i];
                    jointConstraint.position = joint_states_[i];
                    jointConstraint.weight = 1.0;
                    goal_constraints.joint_constraints.push_back(jointConstraint);
                }
                msg.request.goal_constraints.push_back(goal_constraints);
            } else if (goal->mode == moveit_state_server_msgs::GoToStoredStateGoal::GO_TO_STORED_END_EFFECTOR_POSE) {
                geometry_msgs::PoseStamped pose;
                pose.pose = pose_;
                pose.header.frame_id = "world";
                std::vector<double> tolerance_pose(3, 0.01);
                std::vector<double> tolerance_angle(3, 0.01);
                moveit_msgs::Constraints pose_goal =
                        kinematic_constraints::constructGoalConstraints(end_effector_, pose, tolerance_pose,
                                                                        tolerance_angle);
                msg.request.goal_constraints.push_back(pose_goal);
            }
            if (as_.isPreemptRequested() || !ros::ok()) {
                ROS_WARN_STREAM("Preempted moveit_state_server action request");
                as_.setPreempted();
            }
            move_ac_->sendGoal(msg);
            while (true) {
                auto msg = ros::topic::waitForMessage<moveit_msgs::MoveGroupActionFeedback>("move_group/feedback");
                if (msg->feedback.state != "PLANNING" && msg->feedback.state != "MONITOR")break;
            }
            switchController(true);
            as_.setSucceeded();
        }
    }

    void MoveitStateServer::preemptCB() {
        ROS_INFO_STREAM("[moveit_state_server] Preempted moveit_state_server.");
    }


}  // namespace moveit_state_server
int main(int argc, char **argv) {
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    moveit_state_server::MoveitStateServer stateServer;

    ros::spin();

    return 0;
}
