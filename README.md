# Moveit State Server

The ROS package `moveit_state_server` offers services to store the current joint angles or the end effector position (
for example, in the
world frame). Later the robot can return to the saved state. For this purpose, the state is given a unique name when it
is saved via a ROS service. Later an action client, which moves to the stored position using moveit, can be called with
the assigned name.
The package uses the moveit_cpp functionality to access the current state and control the robot.

Moreover, the node can store the joint states persistently in a database. Therefore, it makes use of the mongo database
included in moveit_warehouse. Configured correctly, it stores the joint states in the same database as the moveit motion
planning rviz plugin. Alternatively, the joint states can be persistently stored by saving them as serialized
sensor_msgs/JointState. The parameter `use_databse` in the `moveit_state_server.launch` file decides which persistent
joint state storage is used.

### Launching

If you want to make use of persistently storing joint states in the database, the mongo database must be running. This
can
be done by selecting the `db` param while launching moveit
,
see [moveit documentation: Persistent States](https://ros-planning.github.io/moveit_tutorials/doc/persistent_scenes_and_states/persistent_scenes_and_states.html)
.
Alternatively, the database can be launched with the `database.launch` file in the moveit_state_server package.
If using the moveit launch file, make sure to select the mongo database plugin and selecting the
same port and hostname as in the launch file for the moveit state server.

```bash
roslaunch moveit_state_server database.launch
```

Launching the moveit_state_server can be done with

```bash
roslaunch moveit_state_server moveit_state_server.launch
```

### Storing joint states / end-effector poses

The service `/store_arm_poses` stores either the
current joint angles or the end-effector pose depending on the selected mode. Additionally, the user must provide a name
for the stored state.

#### Service Message Description: Store state

```
int32 mode
string name
int32 STORE_JOINT_POSITIONS=0
int32 STORE_END_EFFECTOR_POSE=1
---
std_msgs/Bool success
```

Note that storing the end-effector pose persistently is not very useful because after restarting the robot the
coordinate system will be different. Hence, only the joint states are stored persistently.

### Retrieving stored joint states / end-effector poses

The service `/retrieve_arm_poses` can be used to retrieve a stored joint_state or end-effector pose.

#### Service Message Description: Retrieve state

```
int32 mode
int32 RETRIEVE_JOINT_POSITIONS=0
int32 RETRIEVE_END_EFFECTOR_POSE=1
string name
---
sensor_msgs/JointState joint_state
geometry_msgs/PoseStamped pose

```

### Moving the robot to a stored state

The action client `/move_arm_to_stored_pose` can be used to move the robot such that it is again in a previously stored
state.
The goal message defines the type and name of the goal

#### Action Message Description: Move arm

```
int32 mode
string name
int32 GO_TO_STORED_JOINT_POSITIONS=0
int32 GO_TO_STORED_END_EFFECTOR_POSE=1
---
int32 success
---
int32 state
```

### Requirements

This ROS package requires `moveit`, [see here](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html), to be installed

If you want to store states persistently with the database, the `warehouse-ros-mongo` package must be installed.
```bash
sudo apt install ros-noetic-warehouse-ros-mongo
```

### Contributing

If you find a bug or would like to contribute to this ROS package, please open an issue on the Github repository or
submit a pull request.

