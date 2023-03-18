#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_state_server/moveit_state_server.h>
#include <controller_manager_msgs/SwitchController.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_state_server/joint_state_storage_database.h>
#include <moveit_state_server/joint_state_file_storage.h>
// controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

namespace moveit_state_server {
    MoveitStateServer::MoveitStateServer(ros::NodeHandle &pnh) : as_(nh_, std::string(ROS_PACKAGE_NAME) +
                                                                          "/move_arm_to_stored_pose",
                                                                     [this](auto &&PH1) {
                                                                         goalCB(std::forward<decltype(PH1)>(PH1));
                                                                     }, false) {
        // SET PARAMETERS FROM LAUNCH FILE
        pnh.param("planning_group", planning_group_, std::string("arm_group"));
        pnh.param("pose_reference_frame", position_reference_frame_, std::string("world"));
        pnh.param("robot_name", robot_name_, std::string(""));
        std::string hostname;
        int port;
        pnh.param("hostname", hostname, std::string("localhost"));
        pnh.param("port", port, 33829);
        std::string folder_path;
        pnh.param("folder_path", folder_path,std::string(""));
        pnh.param("use_database",use_database_for_persistent_storage_,true);
        // SETUP SERVICES FOR STORING AND RETRIEVING STATES AND FOR SWITCHING THE ARM CONTROLLER
        store_pose_service_name_ = "/store_arm_pose";
        retrieve_pose_service_name_ = "/retrieve_arm_pose";
        store_pose_service = pnh.advertiseService(store_pose_service_name_, &MoveitStateServer::storePoseService,
                                                  this);
        retrieve_pose_server =
                pnh.advertiseService(retrieve_pose_service_name_, &MoveitStateServer::retrievePoseService, this);
        switch_controllers_ = nh_.serviceClient<controller_manager_msgs::SwitchController>(
                "/manipulator_arm_control/"
                "controller_manager/"
                "switch_controller");

        // SETUP MOVEIT_CPP
        moveit_cpp_ptr_ = std::make_shared<moveit_cpp::MoveItCpp>(pnh);
        moveit_cpp_ptr_->getPlanningSceneMonitorNonConst()->providePlanningSceneService();
        planning_components_ = std::make_shared<moveit_cpp::PlanningComponent>(planning_group_, moveit_cpp_ptr_);
        auto robot_model_ptr = moveit_cpp_ptr_->getRobotModel();
        auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(planning_group_);
        joint_names_ = joint_model_group_ptr->getActiveJointModelNames();
        end_effector_ = joint_model_group_ptr->getLinkModelNames().back();

        // PRINT JOINT NAMES AND THE END EFFECTOR OF THE SELECTED PLANNING GROUP
        std::stringstream ss;
        ss << "available joints: ";
        for (const auto &elm: joint_names_) ss << elm << ", ";
        ss << "; end effector: " << end_effector_;
        ROS_INFO_STREAM(ss.str());

        // START ACTION SIMPLE ACTION CLIENT
        as_.registerPreemptCallback([this] { preemptCB(); });
        as_.start();

        // SETUP PERSISTENT JOINT STATE STORAGE
        if(use_database_for_persistent_storage_){
            joint_state_storage_ = std::make_unique<joint_storage::JointStateStorageDatabase>(hostname, port, robot_name_);
            joint_state_storage_->loadAllJointStates();
        }else{
            ROS_WARN("initializing file storage.");
            joint_state_storage_ = std::make_unique<joint_storage::JointStateFileStorage>(folder_path, robot_name_);
            joint_state_storage_->loadAllJointStates();
        }

    }

    void MoveitStateServer::storeCurrentJointStates(const std::string &name) {
        sensor_msgs::JointState joint_state;
        joint_state.position.resize(joint_names_.size());
        auto current_state = moveit_cpp_ptr_->getCurrentState();
        current_state->copyJointGroupPositions(planning_group_, joint_state.position);
        joint_state.name = joint_names_;
        joint_state_storage_->addJointState(joint_state, name);
    }


    void MoveitStateServer::storeCurrentPose(const std::string &name) {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = moveit_cpp_ptr_->getTFBuffer()->lookupTransform("world", end_effector_, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
        geometry_msgs::PoseStamped pose;
        convert(transformStamped.transform, pose, position_reference_frame_);
        if (poses_.find(name) != poses_.end()) {
            ROS_WARN_STREAM("The joint_state " << name << " already exists. The value will be overwritten.");
            poses_[name] = pose;
        } else {
            poses_.insert(std::make_pair(name, pose));
        }
    }

    bool MoveitStateServer::storePoseService(moveit_state_server_msgs::StorePoseRequest &req,
                                             moveit_state_server_msgs::StorePoseResponse &res) {
        if (req.mode == moveit_state_server_msgs::StorePoseRequest::STORE_JOINT_POSITIONS) {
            storeCurrentJointStates(req.name);
        }

        if (req.mode == moveit_state_server_msgs::StorePoseRequest::STORE_END_EFFECTOR_POSE) {
            storeCurrentPose(req.name);
        }

        res.success.data = true;
        return true;
    }

    bool MoveitStateServer::retrievePoseService(moveit_state_server_msgs::RetrievePoseRequest &req,
                                                moveit_state_server_msgs::RetrievePoseResponse &res) {
        if (req.mode == moveit_state_server_msgs::RetrievePoseRequest::RETRIEVE_END_EFFECTOR_POSE) {
            geometry_msgs::PoseStamped pose;
            auto it = poses_.find(req.name);
            if (it != poses_.end()) res.pose = it->second;
            else
                ROS_WARN_STREAM("No end_effector pose " << req.name << " stored.");
        } else {
            sensor_msgs::JointState joint_state;
            bool found = joint_state_storage_->getStoredJointState(req.name, joint_state, false);
            if (found)res.joint_state = joint_state;
            else
                ROS_WARN_STREAM("No joint_state " << req.name << " stored.");
        }
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


    void MoveitStateServer::goToStoredJointState(const std::string &name) {
        moveit::core::RobotStatePtr robot_state = moveit_cpp_ptr_->getCurrentState();
        planning_components_->setStartStateToCurrentState();
        sensor_msgs::JointState joint_state;
        if (joint_state_storage_->getStoredJointState(name, joint_state, false)) {
            robot_state->setVariableValues(joint_state);
            planning_components_->setGoal(*robot_state);
            planning_components_->plan();
            planning_components_->execute();
        } else {
            ROS_WARN("Joint State is not saved in database");
        }


    }

    void MoveitStateServer::goToStoredEndeffectorPosition(const std::string &name) {
        planning_components_->setStartStateToCurrentState();
        planning_components_->setGoal(poses_[name], end_effector_);
        planning_components_->plan();
        planning_components_->execute();
    }

    void MoveitStateServer::goalCB(const moveit_state_server_msgs::GoToStoredStateGoalConstPtr &goal) {
        //verify that the named pose has been previously stored
        if (goal->mode == moveit_state_server_msgs::GoToStoredStateGoal::GO_TO_STORED_JOINT_POSITIONS and
            !joint_state_storage_->isJointStateStored(goal->name, true)) {
            ROS_WARN_STREAM(
                    "[moveit_state_server] Before moving the arm to stored joint states, joint positions must be stored by calling the service "
                            << store_pose_service_name_);
            as_.setAborted();
            return;
        }
        if (goal->mode == moveit_state_server_msgs::GoToStoredStateGoal::GO_TO_STORED_END_EFFECTOR_POSE and
            poses_.find(goal->name) == poses_.end()) {
            ROS_WARN_STREAM(
                    "[moveit_state_server] Before moving the arm to stored a stored eef pose, the pose must be stored by calling the service "
                            << store_pose_service_name_);
            as_.setAborted();
            return;
        }

        // switch arm controller and move arm depending on selected options
        if (switchController(false)) {
            if (goal->mode == moveit_state_server_msgs::GoToStoredStateGoal::GO_TO_STORED_JOINT_POSITIONS) {
                goToStoredJointState(goal->name);
            } else {
                goToStoredEndeffectorPosition(goal->name);
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
    ros::NodeHandle pnh_("~");
    std::vector<std::string> test;
    pnh_.getParam("planning_pipelines/pipeline_names", test);
    moveit_state_server::MoveitStateServer stateServer(pnh_);


    ros::spin();

    return 0;
}
