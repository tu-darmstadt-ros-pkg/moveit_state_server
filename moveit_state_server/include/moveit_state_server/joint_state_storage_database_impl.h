//
// Created by aljoscha on 16.03.23.
//

#include <utility>

#include "joint_state_storage_database.h"


namespace joint_storage {


    bool joint_storage::JointStateStorageDatabase::getStoredJointState(const std::string &name,
                                                                       sensor_msgs::JointState &jointState,
                                                                       bool reload) {
        if (reload) loadAllJointStates();
        auto it = joint_states_.find(name);
        if (it != joint_states_.end()) {
            jointState = it->second;
            return true;
        } else {
            ROS_WARN_STREAM("No joint states saved under the name " << name << "!");
            if (!reload)
                return getStoredJointState(name, jointState, true); // reload if not existent, e.g. if stored in rviz
            return false;
        }
    }

    bool joint_storage::JointStateStorageDatabase::storeJointState(sensor_msgs::JointState joint_state,
                                                                   const std::string &name, bool already_exists) {
        moveit_msgs::RobotState robotState;
        robotState.joint_state = joint_state;

        bool success = false;
        // store if db connected
        if (robot_state_storage_) {
            try {
                if (already_exists) robot_state_storage_->removeRobotState(name, robot_name_);
                robot_state_storage_->addRobotState(robotState, name, robot_name_);
                success = true;
            }
            catch (std::exception &ex) {
                ROS_ERROR("Cannot save robot state on the database: %s", ex.what());
            }
        } else {
            ROS_WARN("Warning: Not connected to a database. The state won't be stored persistently.");
        }
        return success;
    }

    bool joint_storage::JointStateStorageDatabase::loadAllJointStates() {
        std::vector<std::string> names;
        std::string pattern = ".*";
        if (robot_state_storage_) {
            try {
                robot_state_storage_->getKnownRobotStates(pattern, names);
            }
            catch (std::exception &ex) {
                ROS_WARN("Cannot query the database""Wrongly formatted regular expression for robot states: ");
                return false;
            }


            for (const std::string &name: names) {
                moveit_warehouse::RobotStateWithMetadata rs;
                bool got_state = false;
                try {
                    got_state = robot_state_storage_->getRobotState(rs, name);
                }
                catch (std::exception &ex) {
                    ROS_ERROR("%s", ex.what());
                }
                if (got_state) {
                    // Overwrite if exists.
                    if (joint_states_.find(name) != joint_states_.end()) {
                        joint_states_.erase(name);
                    }
                    // Store the current start state
                    joint_states_.insert(JointStatePair(name, rs.get()->joint_state));
                }
            }
            return true;
        } else {
            ROS_ERROR("Not connected to the database");
            return false;
        }
    }

    JointStateStorageDatabase::JointStateStorageDatabase(std::string host_name, int port,
                                                         std::string robot_name) :
            JointStateStorage(std::move(robot_name)), host_name_(std::move(host_name)), port_(port) {
        warehouse_ros::DatabaseConnection::Ptr conn = moveit_warehouse::loadDatabase();
        conn->setParams(host_name_, port_, 5.0);
        if (conn->connect()) {
            robot_state_storage_ = std::make_shared<moveit_warehouse::RobotStateStorage>(conn);
            ROS_INFO("Connection to database successful");
        } else {
            ROS_WARN("Connection to database could not be established");
        }
    }
}// namespace joint_storage