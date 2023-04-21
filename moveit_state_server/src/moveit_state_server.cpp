#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_state_server/moveit_state_server.h>
#include <controller_manager_msgs/SwitchController.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_state_server/joint_state_storage_database.h>
#include <moveit_state_server/joint_state_file_storage.h>

namespace moveit_state_server {
    MoveitStateServer::MoveitStateServer(ros::NodeHandle &pnh) : as_(nh_, "/move_arm_to_stored_pose",
                                                                     [this](auto &&PH1) {
                                                                         goalCB(std::forward<decltype(PH1)>(PH1));
                                                                     }, false), pnh_(pnh) {
        // SET PARAMETERS FROM LAUNCH FILE
        pnh.param("pose_reference_frame", position_reference_frame_, std::string("world"));
        pnh.param("robot_name", robot_name_, std::string(""));
        pnh.param("hostname", hostname_, std::string("localhost"));
        pnh.param("port", port_, 33829);
        pnh.param("folder_path", folder_path_, std::string(""));
        pnh.param("use_database", use_database_for_persistent_storage_, true);
        pnh.param("use_move_group_for_moving_robot", use_move_group_for_movement_, true);
        // SETUP SERVICES FOR STORING AND RETRIEVING STATES AND FOR SWITCHING THE ARM CONTROLLER
        store_pose_service_name_ = "/store_arm_pose";
        retrieve_pose_service_name_ = "/retrieve_arm_pose";
        store_pose_service = pnh.advertiseService(store_pose_service_name_, &MoveitStateServer::storePoseService,
                                                  this);
        retrieve_pose_server =
                pnh.advertiseService(retrieve_pose_service_name_, &MoveitStateServer::retrievePoseService, this);
        switch_controllers_ = nh_.serviceClient<controller_manager_msgs::SwitchController>(
                "/manipulator_arm_control/controller_manager/switch_controller");
        // delay moveit initialization to first use of moveit_state_server
        // avoids crashing at start, possibly due to race condition with controllers
        // resetMoveit();

        // START SIMPLE ACTION CLIENT
        as_.registerPreemptCallback([this] { preemptCB(); });
        as_.start();

        // Action Client for move group actions
        move_ac_ = std::make_unique<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>>("move_group");

        // SETUP DYNAMIC RECONFIGURE
        config_server_.setCallback([this](auto &&PH1, auto &&PH2) {
            configCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
        });
    }

    void MoveitStateServer::configCallback(const moveit_state_server::MoveitStateServerConfig &config, uint32_t level){
        if(config.pose_reference_frame !=position_reference_frame_ ){
            ROS_INFO("[MoveitStateServer::configCallback] Changed pose_reference_frame. Deleting stored positions.");
            poses_.clear();
            position_reference_frame_ = config.pose_reference_frame;
        }
        if(config.planning_group != planning_group_){
            ROS_INFO("[MoveitStateServer::configCallback] Change Planning_group. Resetting moveit variables");
            planning_group_ = config.planning_group;
            resetMoveit();
        }
        bool reset_file_storage = (use_database_for_persistent_storage_ != config.use_database_vs_filestorage) ||
                (use_database_for_persistent_storage_ && (config.hostname != hostname_ || config.port != port_)) ||
                (!use_database_for_persistent_storage_ && config.folder_path != folder_path_);
        folder_path_ = config.folder_path;
        hostname_ = config.hostname;
        port_ = config.port;
        use_database_for_persistent_storage_ = config.use_database_vs_filestorage;
        if(reset_file_storage){
            ROS_INFO("[MoveitStateServer::configCallback] Resetting Joint State Storage.");
            resetJointStateStorage();
        }
        // PLANNING PARAMETERS
        plan_request_params_.max_velocity_scaling_factor = config.max_velocity_scaling_factor;
        plan_request_params_.max_acceleration_scaling_factor = config.max_acceleration_scaling_factor;
        plan_request_params_.planning_time = config.planning_time;
        plan_request_params_.planning_attempts = config.planning_attempts;
        ROS_INFO_STREAM("max_velocity_scaling_factor: "<<plan_request_params_.max_velocity_scaling_factor);
        use_move_group_for_movement_ = config.use_move_group_for_movements;
    }

    void MoveitStateServer::initialize() {
        // SETUP MOVEIT_CPP
        resetMoveit();
        // SETUP PERSISTENT JOINT STATE STORAGE EITHER DATABASE OR FILE STORAGE
        resetJointStateStorage();
        plan_request_params_.load(pnh_);
        initialized_ = true;
    }

    void MoveitStateServer::resetJointStateStorage(){
        // SETUP PERSISTENT JOINT STATE STORAGE EITHER DATABASE OR FILE STORAGE
        if (use_database_for_persistent_storage_) {
            ROS_INFO("Initializing Database Joint Storage.");
            joint_state_storage_ = std::make_unique<joint_storage::JointStateStorageDatabase>(hostname_, port_,
                                                                                              robot_name_);
            joint_state_storage_->loadAllJointStates();
        } else {
            ROS_INFO("Initializing File Joint Storage.");
            joint_state_storage_ = std::make_unique<joint_storage::JointStateFileStorage>(folder_path_, robot_name_);
            joint_state_storage_->loadAllJointStates();
        }
    }
    void MoveitStateServer::resetMoveit() {
        ROS_INFO("Setup Moveit #25");
        // SETUP MOVEIT_CPP - make sure that params are on
        moveit_cpp_ptr_.reset(new moveit_cpp::MoveItCpp(pnh_));
        //moveit_cpp_ptr_->getPlanningSceneMonitorNonConst()->providePlanningSceneService();
        // service call to get planning scene, eg. before every move action
        //moveit_cpp_ptr_->getPlanningSceneMonitorNonConst()->requestPlanningSceneState();
        // or subscrive to planning scene topic
        moveit_cpp_ptr_->getPlanningSceneMonitorNonConst()->startSceneMonitor();
        ROS_INFO_STREAM("Planning Scene Topic: "<<
            moveit_cpp_ptr_->getPlanningSceneMonitorNonConst()->DEFAULT_PLANNING_SCENE_TOPIC);
        loadPlanningGroup();
        ROS_INFO("Finished Moveit Setup");
    }

    void MoveitStateServer::loadPlanningGroup() {
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
        ROS_INFO("Finished Planning Group Setup");
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
        // initialize moveit_cpp and JointStorage before first use
        if (!initialized_) initialize();
        // check if planning group still the same else load planning group
        if(planning_group_ != req.planning_group){
            planning_group_ = req.planning_group;
            loadPlanningGroup();
        }
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
        // initialize moveit_cpp and JointStorage before first use
        if (!initialized_) initialize();
        // check if planning group still the same else load planning group
        if(planning_group_ != req.planning_group){
            planning_group_ = req.planning_group;
            loadPlanningGroup();
        }
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
        switchController.request.strictness = switchController.request.BEST_EFFORT;
        return switch_controllers_.call(switchController);
    }

    bool MoveitStateServer::verifyJointStateMoveGroupCompatibility(const sensor_msgs::JointState &jointState)const {
        return joint_names_.size()==jointState.name.size() && joint_names_[0] == jointState.name[0]
        && joint_names_.back() == jointState.name.back();
    }
    bool MoveitStateServer::goToStoredJointState(const std::string &name) {
        moveit::core::RobotStatePtr robot_state = moveit_cpp_ptr_->getCurrentState();
        planning_components_->setStartStateToCurrentState();
        sensor_msgs::JointState joint_state;
        if (joint_state_storage_->getStoredJointState(name, joint_state, false)) {
            if(use_move_group_for_movement_){
                moveit_msgs::Constraints goal_constraints;
                for (int i = 0; i < joint_names_.size(); i++)
                {
                    moveit_msgs::JointConstraint jointConstraint;
                    jointConstraint.joint_name = joint_state.name[i];
                    jointConstraint.position = joint_state.position[i];
                    jointConstraint.weight = 1.0;
                    goal_constraints.joint_constraints.push_back(jointConstraint);
                }
                 goal_.goal.request.goal_constraints.push_back(goal_constraints);
            }else {
                if (!verifyJointStateMoveGroupCompatibility(joint_state)) {
                    ROS_ERROR_STREAM("Stored Joint states names do not match joint names" << joint_state.name[0] <<
                                                                                          " of the selected planning group "
                                                                                          << planning_group_ << "!");
                    return false;
                }
                ROS_INFO("Verified move group joint state msg compatibility");
                robot_state->setVariableValues(joint_state);
                planning_components_->setGoal(*robot_state);
                auto response = planning_components_->plan(plan_request_params_);
                if(response.error_code_ != moveit::core::MoveItErrorCode::SUCCESS) return false;
                return planning_components_->execute();
            }
        } else {
            ROS_WARN("Joint State is not saved in database");
        }
        return true;
    }

    bool MoveitStateServer::goToStoredEndeffectorPosition(const std::string &name) {
        if(use_move_group_for_movement_){
            std::vector<double> tolerance_pose(3, 0.01);
            std::vector<double> tolerance_angle(3, 0.01);
            moveit_msgs::Constraints pose_goal =
                    kinematic_constraints::constructGoalConstraints(end_effector_, poses_[name], tolerance_pose, tolerance_angle);
            goal_.goal.request.goal_constraints.push_back(pose_goal);
        }else{
            planning_components_->setStartStateToCurrentState();
            planning_components_->setGoal(poses_[name], end_effector_);
            auto response = planning_components_->plan(plan_request_params_);
            if(response.error_code_ != moveit::core::MoveItErrorCode::SUCCESS) return false;
            return planning_components_->execute();
        }
        return true;
    }

    void MoveitStateServer::goalCB(const moveit_state_server_msgs::GoToStoredStateGoalConstPtr &goal) {
        // initialize moveit_cpp and JointStorage before first use
        if (!initialized_) initialize();
        // check if planning group still the same else load planning group
        if(planning_group_ != goal->planning_group){
            planning_group_ = goal->planning_group;
            loadPlanningGroup();
        }
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
            // check if moveit thinks controllers are active
            // strange bug; sometimes controllers are active, but moveit thinks they are inactive
            // resetting moveit_cpp_ptr seems to help
            int counter = 0;
            while (!moveit_cpp_ptr_->getTrajectoryExecutionManager()->ensureActiveControllersForGroup(planning_group_) &&
                   counter < 5) {
                ROS_ERROR("Controllers seem to be inactive #44");
                ROS_WARN("Resetting moveit params");
                resetMoveit();
                counter++;
            }
            if(use_move_group_for_movement_){
                goal_ = moveit_msgs::MoveGroupActionGoal(); // reset old constraints
                goal_.goal.planning_options.plan_only = false; // plan and move arrm
                goal_.goal.planning_options.look_around = false;
                goal_.goal.planning_options.replan = false;
                goal_.goal.planning_options.planning_scene_diff.is_diff = true;
                goal_.goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
                goal_.goal.request.start_state.is_diff = true;
                goal_.goal.request.allowed_planning_time = planning_time_;
                goal_.goal.request.num_planning_attempts = planning_attempts_;
                goal_.goal.request.group_name = goal->planning_group;
                goal_.goal.request.start_state.joint_state.name = joint_names_;
                planning_components_->setStartStateToCurrentState();
                std::vector<double> current_joint_states;
                planning_components_->getStartState()->copyJointGroupPositions(goal->planning_group,current_joint_states);
                goal_.goal.request.start_state.joint_state.position = current_joint_states;
                goal_.goal.request.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;
                goal_.goal.request.max_velocity_scaling_factor = max_velocity_scaling_factor_;
            }
            bool successful_movement;
            if (goal->mode == moveit_state_server_msgs::GoToStoredStateGoal::GO_TO_STORED_JOINT_POSITIONS) {
                successful_movement = goToStoredJointState(goal->name);
            } else {
                successful_movement = goToStoredEndeffectorPosition(goal->name);
            }
            if(use_move_group_for_movement_){
                auto state = move_ac_->sendGoalAndWait(goal_.goal);
                successful_movement = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
            }
            switchController(true);
            if(successful_movement){
                as_.setSucceeded();
            }else{
                as_.setAborted();
            }

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
