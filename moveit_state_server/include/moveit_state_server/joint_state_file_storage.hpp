//
// Created by aljoscha on 18.03.23.
//

#ifndef MOVEIT_STATE_SERVER_JOINT_STATE_FILE_STORAGE_H
#define MOVEIT_STATE_SERVER_JOINT_STATE_FILE_STORAGE_H

#include <fstream>
#include <moveit_state_server/joint_state_storage.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace joint_storage
{

class JointStateFileStorage : public JointStateStorage
{
public:
  explicit JointStateFileStorage( std::string folder_path, std::string robot_name );

  bool getStoredJointState( const std::string &name, sensor_msgs::msg::JointState &jointState,
                            bool reload ) override;

  bool storeJointState( const sensor_msgs::msg::JointState &joint_state,
                        const std::string &name ) override;

  bool loadAllJointStates() override;

  void loadJointState( const std::string &path, const std::string &name );

  void testIfDirectoryExistsAndCreateIfNecessary();

private:
  std::string folder_path_;
  std::string extension_ = ".joint_state";
};

} // namespace joint_storage

#endif // MOVEIT_STATE_SERVER_JOINT_STATE_FILE_STORAGE_H
