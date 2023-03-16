#include <sensor_msgs/JointState.h>

class JointStateStorage{
public:
    JointStateStorage();
    virtual void getAllStoredNames(std::vector<std::string> &names, bool reload = false) = 0;
    virtual bool getStoredJointState(const std::string& name, sensor_msgs::JointState& jointState, bool reload = false) = 0;
    virtual bool storeJointState(sensor_msgs::JointState joint_state, const std::string &name) = 0;
    virtual bool loadAllJointState() = 0;
private:
    std::map<std::string, sensor_msgs::JointState> joint_states_;
};