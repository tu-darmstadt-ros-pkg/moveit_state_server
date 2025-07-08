/*  test/joint_state_storage_test.cpp
 *  Unit tests for JointState*Storage back-ends (ROS 2 version)
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include <moveit_state_server/joint_state_file_storage.hpp>

using joint_storage::JointStateStorage;
namespace fs = std::filesystem;

/* ------------------------------------------------------------------ */
/* Helper test routines (unchanged except for msg namespace)          */
/* ------------------------------------------------------------------ */
void testGetStoredJointState(const std::shared_ptr<JointStateStorage> &storage)
{
  sensor_msgs::msg::JointState js;
  js.name      = {"joint1", "joint2", "joint3"};
  js.position  = {1.0, 2.0, 3.0};

  ASSERT_TRUE(storage->addJointState(js, "test_joint"));

  sensor_msgs::msg::JointState out;
  ASSERT_TRUE(storage->getStoredJointState("test_joint", out, true));
  EXPECT_EQ(js.name,     out.name);
  EXPECT_EQ(js.position, out.position);
}

void testOverwriteJointState(const std::shared_ptr<JointStateStorage> &storage)
{
  sensor_msgs::msg::JointState js1, js2;
  js1.name = {"joint1"}; js1.position = {1.0}; js1.velocity = {2.0};
  js2.name = {"joint1"}; js2.position = {3.0}; js2.velocity = {4.0};

  ASSERT_TRUE(storage->addJointState(js1, "test_joint"));
  ASSERT_TRUE(storage->addJointState(js2, "test_joint"));   // overwrite

  sensor_msgs::msg::JointState out;
  ASSERT_TRUE(storage->getStoredJointState("test_joint", out, false));

  EXPECT_EQ(js2.name,     out.name);
  EXPECT_EQ(js2.position, out.position);
  EXPECT_EQ(js2.velocity, out.velocity);
}

void testReloadedJointStates(const std::shared_ptr<JointStateStorage> &storage,
                             const sensor_msgs::msg::JointState &js,
                             const std::string &name)
{
  storage->loadAllJointStates();

  sensor_msgs::msg::JointState out;
  ASSERT_TRUE(storage->getStoredJointState(name, out, true));
  EXPECT_EQ(js.name,     out.name);
  EXPECT_EQ(js.position, out.position);
}

/* ------------------------------------------------------------------ */
/*  File-storage helpers                                              */
/* ------------------------------------------------------------------ */
static std::shared_ptr<JointStateStorage>
initializeFileStorage(bool clear_content = false)
{
  const std::string folder = "/tmp/moveit_state_server_test";
    /*
      ament_index_cpp::get_package_share_directory("moveit_state_server") +
      "/test/default_test_file_storage";*/

  if (clear_content && fs::exists(folder))
    fs::remove_all(folder);

  return std::make_shared<joint_storage::JointStateFileStorage>(folder,
                                                                "athena");
}

/* ------------------------------------------------------------------ */
/*  GTests                                                             */
/* ------------------------------------------------------------------ */
TEST(JointStateFileStorage, GetStoredJointState_FILE)
{
  auto storage = initializeFileStorage(true);
  testGetStoredJointState(storage);
}

TEST(JointStateFileStorage, OverwriteJointState_FILE)
{
  auto storage = initializeFileStorage(true);
  testOverwriteJointState(storage);
}

TEST(JointStateFileStorage, ReloadFromDisk_FILE)
{
  auto storage = initializeFileStorage(true);

  sensor_msgs::msg::JointState js;
  js.name = {"joint1", "joint2", "joint3"};
  js.position = {1.0, 2.0, 3.0};
  std::string name = "test_joint";

  storage->addJointState(js, name);
  storage.reset();                         // destroy instance -> flush file

  storage = initializeFileStorage();       // new instance -> reload
  testReloadedJointStates(storage, js, name);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);        // needed for node handles inside storage
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
