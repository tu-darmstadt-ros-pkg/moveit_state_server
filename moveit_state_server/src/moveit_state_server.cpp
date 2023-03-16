#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_state_server/moveit_state_server.h>
#include <controller_manager_msgs/SwitchController.h>
#include <moveit/kinematic_constraints/utils.h>
// controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

namespace moveit_state_server {
    MoveitStateServer::MoveitStateServer(ros::NodeHandle &pnh) : as_(nh_, "/move_arm_to_stored_pose",
                                                 [this](auto &&PH1) { goalCB(std::forward<decltype(PH1)>(PH1)); },
                                                 false) {
            // TODO: -get params like arm_group,  from launch file
            // TODO: - store maps of string and jointstates/poses
            // TODO: change msg for services to include names
            // TODO: include persistent joint_state_database access

            store_pose_service_name_ = "/store_arm_joint_states";
            store_pose_service = pnh.advertiseService(store_pose_service_name_, &MoveitStateServer::storePoseService,
                                                       this);
            retrieve_pose_server =
                    pnh_.advertiseService("/retrieve_arm_joint_states", &MoveitStateServer::retrievePoseService, this);
            get_planning_scene_ = pnh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
            switch_controllers_ = nh_.serviceClient<controller_manager_msgs::SwitchController>(
                    "/manipulator_arm_control/"
                    "controller_manager/"
                    "switch_controller");

            moveit_cpp_ptr_ = std::make_shared<moveit_cpp::MoveItCpp>(pnh);
            moveit_cpp_ptr_->getPlanningSceneMonitorNonConst()->providePlanningSceneService();

            planning_components_ = std::make_shared<moveit_cpp::PlanningComponent>(planning_group_, moveit_cpp_ptr_);
            auto robot_model_ptr = moveit_cpp_ptr_->getRobotModel();
            auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(planning_group_);
            joint_names_ = joint_model_group_ptr->getActiveJointModelNames();
            end_effector_ = joint_model_group_ptr->getLinkModelNames().back();
            std::stringstream ss;
            ss<<"available joints: ";
            for(const auto& elm:joint_names_) ss<<elm<<", ";
            ss<<"; end effector: "<<end_effector_;
            ROS_INFO_STREAM(ss.str());
            as_.registerPreemptCallback([this] { preemptCB(); });
            as_.start();
       // }
    }

    void MoveitStateServer::storeCurrentJointStates(const std::string& name) {
        std::vector<double> joint_states(joint_names_.size());
        auto current_state = moveit_cpp_ptr_->getCurrentState();
        current_state->copyJointGroupPositions(planning_group_, joint_states);
        if(joint_states_.find(name)!=joint_states_.end()){
            ROS_WARN_STREAM("The joint_state "<<name<<" already exists. The value will be overwritten.");
            joint_states_[name] = joint_states;
        }else{
            joint_states_.insert(std::make_pair(name, joint_states));
        }
    }


    void MoveitStateServer::storeCurrentPose(const std::string& name) {

        //check if current pose already exists
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = moveit_cpp_ptr_->getTFBuffer()->lookupTransform("world", end_effector_, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
        geometry_msgs::PoseStamped pose;
        convert(transformStamped.transform, pose,position_reference_frame_);
        if(poses_.find(name)!=poses_.end()){
            ROS_WARN_STREAM("The joint_state "<<name<<" already exists. The value will be overwritten.");
            poses_[name] = pose;
        }else{
            poses_.insert(std::make_pair(name, pose));
        }
    }

    bool MoveitStateServer::storePoseService(moveit_state_server_msgs::StorePoseRequest &req,
                                             moveit_state_server_msgs::StorePoseResponse &res) {
        if (req.mode == moveit_state_server_msgs::StorePoseRequest::STORE_JOINT_POSITIONS) {
            storeCurrentJointStates(req.name);
            stored_joint_positions_ = true;
        }

        if (req.mode == moveit_state_server_msgs::StorePoseRequest::STORE_END_EFFECTOR_POSE) {
            storeCurrentPose(req.name);
            stored_end_effector_position_ = true;
        }

        res.success.data = true;
        return true;
    }

    bool MoveitStateServer::retrievePoseService(moveit_state_server_msgs::RetrievePoseRequest &req,
                                                moveit_state_server_msgs::RetrievePoseResponse &res) {
        res.joint_states = joint_states_[""]; // TODO
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
    //check before if
    void MoveitStateServer::go_to_stored_joint_state(const std::string& name){
        moveit::core::RobotStatePtr robot_state = moveit_cpp_ptr_->getCurrentState();
        planning_components_->setStartStateToCurrentState();
        robot_state->setVariablePositions(joint_names_,joint_states_[name]);
        planning_components_->setGoal(*robot_state);
        planning_components_->plan();
        planning_components_->execute();

    }

    void MoveitStateServer::go_to_stored_eef_position(const std::string& name){
        planning_components_->setStartStateToCurrentState();
        planning_components_->setGoal(poses_[name], end_effector_);
        planning_components_->plan();
        planning_components_->execute();
    }

    void MoveitStateServer::goalCB(const moveit_state_server_msgs::GoToStoredStateGoalConstPtr &goal) {
        //verify that the named pose has been previously stored
        if (goal->mode == moveit_state_server_msgs::GoToStoredStateGoal::GO_TO_STORED_JOINT_POSITIONS and
            joint_states_.find(goal->name)==joint_states_.end()) {
            ROS_WARN_STREAM(
                    "[moveit_state_server] Before moving the arm to stored joint states, joint positions must be stored by calling the service "
                            << store_pose_service_name_);
            as_.setAborted();
            return;
        }
        if (goal->mode == moveit_state_server_msgs::GoToStoredStateGoal::GO_TO_STORED_END_EFFECTOR_POSE and
                poses_.find(goal->name)==poses_.end()) {
            ROS_WARN_STREAM(
                    "[moveit_state_server] Before moving the arm to stored a stored eef pose, the pose must be stored by calling the service "
                            << store_pose_service_name_);
            as_.setAborted();
            return;
        }

        // switch arm controller and move arm depending on selected options
        if (switchController(false)) {
            if(goal->mode == moveit_state_server_msgs::GoToStoredStateGoal::GO_TO_STORED_JOINT_POSITIONS){
                go_to_stored_joint_state(goal->name);
            }else{
                go_to_stored_eef_position(goal->name);
            }
            switchController(true);
            as_.setSucceeded();
        }
    }

    void MoveitStateServer::preemptCB() {
        ROS_WARN("[moveit_state_server] Preempted moveit_state_server.");
        moveit_cpp_ptr_->getTrajectoryExecutionManager()->stopExecution();
        ROS_WARN("[moveit_state_server] Stopping trajectory execution...");
    }


}  // namespace moveit_state_server
int main(int argc, char **argv) {
    ros::init(argc, argv, "moveit_state_server");
    ros:: NodeHandle pnh_("~");
    std::vector<std::string> test;
    pnh_.getParam("planning_pipelines/pipeline_names", test);
    moveit_state_server::MoveitStateServer stateServer(pnh_);


    ros::spin();

    return 0;
}
