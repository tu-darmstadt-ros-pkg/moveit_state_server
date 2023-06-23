//
// Created by aljoscha on 16.03.23.
//

#ifndef MOVEIT_STATE_SERVER_JOINT_STATE_STORAGE_H
#define MOVEIT_STATE_SERVER_JOINT_STATE_STORAGE_H

#include <sensor_msgs/JointState.h>
# include <ros/ros.h>
#include <utility>

namespace joint_storage {
class JointStateStorage {
public:
  explicit JointStateStorage(std::string robot_name) : robot_name_(std::move(robot_name)) {};

  virtual ~JointStateStorage() = default;

  using JointStatePair = std::pair<std::string, sensor_msgs::JointState>;

  virtual void getAllStoredNames(std::vector<std::string> &names, bool reload) {
    if (reload)loadAllJointStates();
    names.clear();
    for (const auto &pair: joint_states_) {
      names.push_back(pair.first);
    }
  }

  virtual bool getStoredJointState(const std::string &name, sensor_msgs::JointState &jointState, bool reload) = 0;


  virtual bool loadAllJointStates() = 0;

  inline bool isJointStateStored(const std::string &name, bool allow_reloading) {
    if (allow_reloading && joint_states_.find(name) == joint_states_.end())loadAllJointStates();
    return joint_states_.find(name) != joint_states_.end();
  }

  bool addJointState(const sensor_msgs::JointState &joint_state, const std::string &name) {
    // add joint state to map
    auto it = joint_states_.find(name);
    bool already_exists = false;
    if (it != joint_states_.end()) {
      //ROS_WARN_STREAM("The joint_state "<< name << " already exists. The value will be overwritten.");
      it->second = joint_state;
      already_exists = true;
    } else {
      joint_states_.insert(std::make_pair(name, joint_state));
    }
    return storeJointState(joint_state, name, already_exists);
  }

protected:
  virtual bool
  storeJointState(sensor_msgs::JointState joint_state, const std::string &name, bool already_exists) = 0;

  std::map<std::string, sensor_msgs::JointState> joint_states_;
  std::string robot_name_;
};

}// namespace joint_storage
#endif //MOVEIT_STATE_SERVER_JOINT_STATE_STORAGE_H