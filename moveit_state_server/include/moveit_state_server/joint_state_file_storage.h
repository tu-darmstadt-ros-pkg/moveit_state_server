//
// Created by aljoscha on 18.03.23.
//

#ifndef MOVEIT_STATE_SERVER_JOINT_STATE_FILE_STORAGE_H
#define MOVEIT_STATE_SERVER_JOINT_STATE_FILE_STORAGE_H

#include <moveit_state_server/joint_state_storage.h>
#include <fstream>

namespace joint_storage {

    class JointStateFileStorage : public JointStateStorage {
    public:
        explicit JointStateFileStorage(std::string folder_path, std::string robot_name);


        bool getStoredJointState(const std::string &name, sensor_msgs::JointState &jointState, bool reload) override;

        bool
        storeJointState(sensor_msgs::JointState joint_state, const std::string &name, bool already_exists) override;

        bool loadAllJointStates() override;

        void loadJointState(const std::string &path, const std::string &name);

        void testIfDirectoryExistsAndCreateIfNecessary();


    private:
        std::string folder_path_;
        std::string extension_ = ".joint_state";
    };

} // namespace joint_storage
#include "moveit_state_server/joint_state_file_storage_impl.h"

#endif //MOVEIT_STATE_SERVER_JOINT_STATE_FILE_STORAGE_H
