//
// Created by aljoscha on 16.03.23.
//

#ifndef MOVEIT_STATE_SERVER_JOINT_STATE_STORAGE_DATABASE_H
#define MOVEIT_STATE_SERVER_JOINT_STATE_STORAGE_DATABASE_H


#include <moveit_state_server/joint_state_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/warehouse/moveit_message_storage.h>

namespace joint_storage {
    class JointStateStorageDatabase : public JointStateStorage {
    public:
        JointStateStorageDatabase(std::string host_name, int port,
                                  std::string robot_name = "");

//        ~JointStateStorageDatabase() {
//            robot_state_storage_->reset();
//        }


        bool
        getStoredJointState(const std::string &name, sensor_msgs::JointState &jointState, bool reload) override;

        bool
        storeJointState(sensor_msgs::JointState joint_state, const std::string &name, bool already_exists) override;

        bool loadAllJointStates() override;


    private:
        moveit_warehouse::RobotStateStoragePtr robot_state_storage_;
        std::string host_name_;
        int port_;
    };
}// namespace joint_storage#

#include "moveit_state_server/joint_state_storage_database_impl.h"

#endif //MOVEIT_STATE_SERVER_JOINT_STATE_STORAGE_DATABASE_H
