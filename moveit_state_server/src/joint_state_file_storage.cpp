//
// Created by aljoscha-schmidt on 7/8/25.
//

#include <moveit_state_server/joint_state_file_storage.hpp>

#include <fstream>
#include <iostream>
#include <filesystem>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

namespace fs = std::filesystem;

namespace joint_storage
{

/* -------------------------------------------------- */
/* ctor & helpers                                     */
/* -------------------------------------------------- */
JointStateFileStorage::JointStateFileStorage(std::string folder_path,
                                             std::string robot_name)
: JointStateStorage(std::move(robot_name))
, folder_path_(std::move(folder_path))
{
  if (!fs::exists(folder_path_))
    fs::create_directories(folder_path_);
}

/* -------------------------------------------------- */
/* Load a single message from disk                    */
/* -------------------------------------------------- */
void JointStateFileStorage::loadJointState(const std::string &file_path,
                                           const std::string &pose_name)
{
  std::ifstream ifs(file_path, std::ios::binary | std::ios::ate);
  if (!ifs) {
    std::cerr << "Cannot open " << file_path << '\n';
    return;
  }

  const std::streamsize size = ifs.tellg();
  ifs.seekg(0, std::ios::beg);

  rclcpp::SerializedMessage serialized_msg(size);
  auto & rcl_msg = serialized_msg.get_rcl_serialized_message();

  if (!ifs.read(reinterpret_cast<char*>(rcl_msg.buffer), size)) {
    std::cerr << "Failed to read " << file_path << '\n';
    return;
  }
  rcl_msg.buffer_length = size;

  rclcpp::Serialization<sensor_msgs::msg::JointState> serializer;
  sensor_msgs::msg::JointState joint_state;
  serializer.deserialize_message(&serialized_msg, &joint_state);

  joint_states_.insert({pose_name, joint_state});
}

/* -------------------------------------------------- */
/* Load *all* joint‑state files in folder             */
/* -------------------------------------------------- */
bool JointStateFileStorage::loadAllJointStates()
{
  if (!fs::is_directory(folder_path_)) {
    std::cerr << "Error: '" << folder_path_ << "' is not a directory.\n";
    return false;
  }

  const std::string prefix = robot_name_ + "_";

  for (const auto &entry : fs::directory_iterator(folder_path_)) {
    if (!entry.is_regular_file())   continue;
    if (entry.path().extension() != extension_) continue;

    const std::string stem = entry.path().stem().string();
    if (stem.rfind(prefix, 0) != 0) continue;               // wrong robot

    const std::string pose_name = stem.substr(prefix.size());
    loadJointState(entry.path().string(), pose_name);
  }
  return true;
}

/* -------------------------------------------------- */
/* Return a stored joint state                        */
/* -------------------------------------------------- */
bool JointStateFileStorage::getStoredJointState(const std::string &name,
                                                sensor_msgs::msg::JointState &out,
                                                bool /*reload*/)
{
  auto it = joint_states_.find(name);
  if (it == joint_states_.end())
    return false;

  out = it->second;
  return true;
}

/* -------------------------------------------------- */
/* Save a joint state to disk                         */
/* -------------------------------------------------- */
bool JointStateFileStorage::storeJointState(const sensor_msgs::msg::JointState &joint_state,
                                            const std::string &name)
{
  const std::string file =
      (fs::path(folder_path_) /
       (robot_name_ + "_" + name + extension_))
          .string();

  rclcpp::Serialization<sensor_msgs::msg::JointState> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(&joint_state, &serialized_msg);

  auto & rcl_msg = serialized_msg.get_rcl_serialized_message();
  std::ofstream ofs(file, std::ios::binary);
  ofs.write(reinterpret_cast<const char*>(rcl_msg.buffer),
            static_cast<std::streamsize>(rcl_msg.buffer_length));
  return static_cast<bool>(ofs);
}

}  // namespace joint_storage

