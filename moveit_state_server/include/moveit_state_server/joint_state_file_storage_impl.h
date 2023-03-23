//
// Created by aljoscha on 18.03.23.
//
#ifndef MOVEIT_STATE_SERVER_JOINT_STATE_FILE_STORAGE_IMPL
#define MOVEIT_STATE_SERVER_JOINT_STATE_FILE_STORAGE_IMPL
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include <boost/filesystem.hpp>
#include <moveit_state_server/joint_state_file_storage.h>
#include "sensor_msgs/JointState.h"

namespace joint_storage {

    void JointStateFileStorage::loadJointState(const std::string &path, const std::string &name) {
        sensor_msgs::JointState jointState;
        //ROS_INFO_STREAM("Reading joint_state for pose " << name << " from path " << path);
        std::ifstream ifs(path, std::ios::in | std::ios::binary);
        ifs.seekg(0, std::ios::end);
        std::streampos end = ifs.tellg();
        ifs.seekg(0, std::ios::beg);
        std::streampos begin = ifs.tellg();

        uint32_t file_size = end - begin;
        boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
        ifs.read((char *) ibuffer.get(), file_size);
        ros::serialization::IStream istream(ibuffer.get(), file_size);
        ros::serialization::deserialize(istream, jointState);
        ifs.close();
        joint_states_.insert(JointStatePair(name, jointState));
    }

    bool JointStateFileStorage::loadAllJointStates() {
        // test if folder exists
        if (!boost::filesystem::exists(folder_path_) || !boost::filesystem::is_directory(folder_path_)) {
            std::cerr << "Error: " << folder_path_ << " is not a valid directory." << std::endl;
            return false;
        }
        // iterate over folder and load all joints states (check for correct robot)
        boost::filesystem::directory_iterator end_itr;  // Default construction yields past-the-end
        for (boost::filesystem::directory_iterator itr(folder_path_); itr != end_itr; ++itr) {
            std::string robot_joint_state_name = itr->path().stem().string();
            if (boost::filesystem::is_regular_file(itr->path()) &&
                itr->path().extension() == extension_ &&
                robot_joint_state_name.find(robot_name_) != std::string::npos) {
                std::cout << itr->path().filename() << std::endl;  // Replace with your desired processing of file path

                robot_joint_state_name.erase(robot_joint_state_name.begin(),
                                             robot_joint_state_name.begin() + robot_name_.length() +
                                             1);
                loadJointState(itr->path().string(), robot_joint_state_name);
            }
        }
        return true;
    }

    bool JointStateFileStorage::getStoredJointState(const std::string &name,
                                                    sensor_msgs::JointState &jointState,
                                                    bool reload) {
        auto it = joint_states_.find(name);
        if (it != joint_states_.end()) {
            jointState = it->second;
            return true;
        } else {
            //ROS_WARN_STREAM("No joint states saved under the name " << name << "!");
            return false;
        }
    }

    bool JointStateFileStorage::storeJointState(sensor_msgs::JointState joint_state, const std::string &name,
                                                bool already_exists) {
        // Write to File
        std::ofstream ofs(folder_path_ + "/" + robot_name_ + "_" + name + extension_, std::ios::out | std::ios::binary);
        uint32_t serial_size = ros::serialization::serializationLength(joint_state);
        boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);
        ros::serialization::OStream ostream(obuffer.get(), serial_size);
        ros::serialization::serialize(ostream, joint_state);
        ofs.write((char *) obuffer.get(), serial_size);
        ofs.close();
        return true;
    }


    JointStateFileStorage::JointStateFileStorage(std::string folder_path, std::string robot_name) : JointStateStorage(
            std::move(robot_name)), folder_path_(std::move(folder_path)) {
        testIfDirectoryExistsAndCreateIfNecessary();
    }

    void JointStateFileStorage::testIfDirectoryExistsAndCreateIfNecessary() {
        if (!boost::filesystem::is_directory(folder_path_)) {
            boost::filesystem::create_directory(folder_path_);
            std::cout << "Directory created!" << std::endl;
        } else {
            std::cout << "Directory already exists." << std::endl;
        }
    }

}  // namespace joint_storage

# endif // MOVEIT_STATE_SERVER_JOINT_STATE_FILE_STORAGE_IMPL