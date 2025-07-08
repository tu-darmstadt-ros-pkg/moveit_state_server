
#ifndef MOVEIT_STATE_SERVER__JOINT_STATE_STORAGE_HPP_
#define MOVEIT_STATE_SERVER__JOINT_STATE_STORAGE_HPP_

#include <sensor_msgs/msg/joint_state.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace joint_storage
{

/**
 * @brief Abstract base class for persistent JointState storage back‑ends.
 *
 * Derived classes implement `loadAllJointStates()` and `storeJointState()`
 * to back the map with files, databases, etc.
 */
class JointStateStorage
{
public:
  explicit JointStateStorage(std::string robot_name)
  : robot_name_(std::move(robot_name)) {}

  virtual ~JointStateStorage() = default;

  using JointStateMsg  = sensor_msgs::msg::JointState;
  using JointStatePair = std::pair<std::string, JointStateMsg>;

  /** @brief Return all stored names. Optionally reload from back‑end. */
  virtual void getAllStoredNames(std::vector<std::string> &names, bool reload)
  {
    if (reload) loadAllJointStates();
    names.clear();
    for (const auto &pair : joint_states_)
      names.push_back(pair.first);
  }

  /** @brief Retrieve a single joint state (pure virtual). */
  virtual bool getStoredJointState(const std::string &name,
                                   JointStateMsg            &joint_state,
                                   bool                      reload) = 0;

  /** @brief Force a full reload from back‑end (pure virtual). */
  virtual bool loadAllJointStates() = 0;

  /** @brief True if the name exists (with optional lazy reload). */
  bool isJointStateStored(const std::string &name, bool allow_reloading)
  {
    if (allow_reloading && joint_states_.count(name) == 0)
      loadAllJointStates();
    return joint_states_.count(name) != 0;
  }

  /**
   * @brief Insert or update a joint state and persist to back‑end.
   * @return true if persistence succeeded.
   */
  bool addJointState(const JointStateMsg &joint_state, const std::string &name)
  {
    bool already_exists = (joint_states_.count(name) != 0);
    joint_states_[name] = joint_state;
    return storeJointState(joint_state, name);
  }

protected:
  /** @brief Persist a single state (pure virtual). */
  virtual bool storeJointState(const JointStateMsg &joint_state,
                               const std::string   &name) = 0;

  std::map<std::string, JointStateMsg> joint_states_;
  std::string robot_name_;
};

}  // namespace joint_storage

#endif  // MOVEIT_STATE_SERVER__JOINT_STATE_STORAGE_HPP_
