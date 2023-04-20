//
// Created by aljoscha on 08.05.22.
//
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
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit_state_server/joint_state_storage.h>

namespace moveit_state_server {
    inline
    void convert(const geometry_msgs::Transform &trans, geometry_msgs::PoseStamped &pose, const std::string &frame) {
        pose.header.frame_id = frame;
        pose.pose.orientation = trans.rotation;
        pose.pose.position.x = trans.translation.x;
        pose.pose.position.y = trans.translation.y;
        pose.pose.position.z = trans.translation.z;
    }

    class MoveitStateServer {
    public:

        explicit MoveitStateServer(ros::NodeHandle &pnh);

    private:
        bool storePoseService(moveit_state_server_msgs::StorePoseRequest &req,
                              moveit_state_server_msgs::StorePoseResponse &res);

        bool retrievePoseService(moveit_state_server_msgs::RetrievePoseRequest &req,
                                 moveit_state_server_msgs::RetrievePoseResponse &res);

        void goalCB(const moveit_state_server_msgs::GoToStoredStateGoalConstPtr &goal);

        void preemptCB();

        bool switchController(bool to_tcp);

        void storeCurrentJointStates(const std::string &name);

        void storeCurrentPose(const std::string &name);

        void goToStoredJointState(const std::string &name);

        void goToStoredEndeffectorPosition(const std::string &name);

        void resetMoveit();

        void initialize();

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        // params for Joint Storage
        int port_;
        std::string folder_path_;
        std::string hostname_;

        std::string planning_group_ = "arm_group";
        std::string position_reference_frame_ = "world";
        std::string robot_name_;
        std::string retrieve_pose_service_name_;
        std::string end_effector_;
        std::string store_pose_service_name_;
        ros::ServiceServer store_pose_service;
        ros::ServiceServer retrieve_pose_server;
        robot_model::RobotModelPtr moveit_robot_model_;
        actionlib::SimpleActionServer<moveit_state_server_msgs::GoToStoredStateAction> as_;
        std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr_;
        std::shared_ptr<moveit_cpp::PlanningComponent> planning_components_;
        ros::ServiceClient get_planning_scene_;
        ros::ServiceClient switch_controllers_;
        std::vector<std::string> joint_names_;
        std::map<std::string, geometry_msgs::PoseStamped> poses_;
        std::unique_ptr<joint_storage::JointStateStorage> joint_state_storage_;
        bool use_database_for_persistent_storage_ = true;
        bool initialized_ = false;
    private:
    };
}
#endif //MOVEIT_STATE_SERVER_H