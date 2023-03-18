
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <moveit_state_server/joint_state_storage_database_impl.h>
#include <moveit_state_server/joint_state_file_storage_impl.h>


void testGetStoredJointState(const std::shared_ptr<joint_storage::JointStateStorage> &storage) {
    // Create a joint state message
    sensor_msgs::JointState joint_state;
    joint_state.name = {"joint1", "joint2", "joint3"};
    joint_state.position = {1.0, 2.0, 3.0};

    // Store the joint state
    EXPECT_TRUE(storage->addJointState(joint_state, "test_joint"));

    // Retrieve the joint state
    sensor_msgs::JointState joint_state_out;
    EXPECT_TRUE(storage->getStoredJointState("test_joint", joint_state_out, true));
    EXPECT_EQ(joint_state.name, joint_state_out.name);
    EXPECT_EQ(joint_state.position, joint_state_out.position);
}

void testOverwriteJointState(const std::shared_ptr<joint_storage::JointStateStorage> &storage) {
    // Create two test joint states with the same name
    sensor_msgs::JointState joint_state1;
    joint_state1.name.push_back("joint1");
    joint_state1.position.push_back(1.0);
    joint_state1.velocity.push_back(2.0);

    sensor_msgs::JointState joint_state2;
    joint_state2.name.push_back("joint1");
    joint_state2.position.push_back(3.0);
    joint_state2.velocity.push_back(4.0);

    // Add the first joint state to the storage
    ASSERT_TRUE(storage->addJointState(joint_state1, "test_joint"));

    // Add the second joint state to the storage, which should overwrite the first joint state
    ASSERT_TRUE(storage->addJointState(joint_state2, "test_joint"));

    // Retrieve the joint state from the storage
    sensor_msgs::JointState retrieved_joint_state;
    ASSERT_TRUE(storage->getStoredJointState("test_joint", retrieved_joint_state, false));

    // Compare the retrieved joint state with the second joint state
    ASSERT_EQ(joint_state2.name[0], retrieved_joint_state.name[0]);
    ASSERT_EQ(joint_state2.position[0], retrieved_joint_state.position[0]);
    ASSERT_EQ(joint_state2.velocity[0], retrieved_joint_state.velocity[0]);
}

void testReloadedJointStates(const std::shared_ptr<joint_storage::JointStateStorage> &storage,
                             sensor_msgs::JointState joint_state, std::string name) {
    // Create a test joint state

    storage->loadAllJointStates();

    // Retrieve the joint state from the storage
    sensor_msgs::JointState joint_state_out;
    EXPECT_TRUE(storage->getStoredJointState(name, joint_state_out, true));
    EXPECT_EQ(joint_state.name, joint_state_out.name);
    EXPECT_EQ(joint_state.position, joint_state_out.position);
}

std::shared_ptr<joint_storage::JointStateStorage> initializeStorageDatabase() {
    std::string hostname = "localhost";
    int port = 33289;
    return std::make_shared<joint_storage::JointStateStorageDatabase>(hostname, port, std::string("asterix"));
}

std::shared_ptr<joint_storage::JointStateStorage> initializeFileStorage() {
    std::string folder_path = ros::package::getPath("moveit_state_server") + "/test/default_test_file_storage";
    return std::make_shared<joint_storage::JointStateFileStorage>(folder_path, std::string("asterix"));
}

TEST(JointStateStorageDatabase, GetStoredJointState) {
    auto storage = initializeStorageDatabase();
    testGetStoredJointState(storage);
}

TEST(JointStateStorageDatabase, OverwriteJointState) {
    auto storage = initializeStorageDatabase();
    testOverwriteJointState(storage);
}

TEST(JointStateStorageDatabase, ReloadFromDatabaseOrFile) {
    auto storage = initializeStorageDatabase();
    sensor_msgs::JointState joint_state;
    std::string name = "test_joint";
    joint_state.name = {"joint1", "joint2", "joint3"};
    joint_state.position = {1.0, 2.0, 3.0};
    storage->addJointState(joint_state, name);
    storage.reset();
    storage = initializeStorageDatabase();
    testReloadedJointStates(storage, joint_state, name);
}

TEST(JointStateFileStorage, GetStoredJointState) {
    auto storage = initializeFileStorage();
    testGetStoredJointState(storage);
}

TEST(JointStateFileStorage, OverwriteJointState) {
    auto storage = initializeFileStorage();
    testOverwriteJointState(storage);
}

TEST(JointStateFileStorage, ReloadFromDatabaseOrFile) {
    auto storage = initializeFileStorage();
    sensor_msgs::JointState joint_state;
    std::string name = "test_joint";
    joint_state.name = {"joint1", "joint2", "joint3"};
    joint_state.position = {1.0, 2.0, 3.0};
    storage->addJointState(joint_state, name);
    storage.reset();
    storage = initializeFileStorage();
    testReloadedJointStates(storage, joint_state, name);
}

//needs to be launched as rostest
// e.g.: rostest moveit_state_server joint_state_storage_test.test
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_joint_storage");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
