# Collision Reward Plugin ROS2


## Introduction

Custom Collision Reward Plugin implementation example for objects in ROS2 with Gazebo

## Dependencies

- OS: Ubuntu Linux 22.04
- ROS Version: ROS2 Humble
- C++

## How to Use

### Create Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Pull and Build Package

```bash
git clone https://github.com/appmdev/reward_collision_plugin_ros2.git
```
**Resolve Dependencies**
```bash
cd ..
rosdep install -i --from-path src --rosdistro humble -y
```

```bash
colcon build --packages-select collision_reward --symlink-install
```
### Launch Collision Reward

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source /usr/share/gazebo-11/setup.bash
ros2 launch collision_reward robot.launch.py
```

### Launch Human with custom plugin

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch collision_reward spawn_human_ros2.launch.py
```

### Spawn Object

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run collision_reward spawn_object.py
```

**Topics to be listed**

```bash
ros2 topic list
```

Output:
```bash
/clock
/obj1/joint_states
/obj1/robot_description
/obj1/tf
/obj1/tf_static
/parameter_events
/performance_metrics
/reward
/rosout
```

### The /reward topic will publish as float "10.00"

# Customise to your object and plugin


### To implement your custom plugin do following

#### Copy and rename accordingly "spawn_human_ros2.launch.py" file and modify with your custom object

```bash
urdf_file = 'custom_name.urdf'
xacro_file = "custom_name.urdf.xacro"
namespace_declared = "obj2"
```
#### Add following to urdf object "custom_name.urdf.xacro" of your custom object
```bash
  <!-- Custom Gazebo Plugin for Collision Reward  -->
  <gazebo>
    <plugin filename="libcollision_reward_plugin.so" name="collision_reward_plugin">
      <target_collision>waffle</target_collision>
      <reward_value>10.0</reward_value>
      <ros>
        <namespace>/human1</namespace>
      </ros>    
    </plugin>
  </gazebo>
```
#### Create folder with custom object name
```bash
mkdir -p ~/ros2_ws/src/reward_collision_plugin_ros2/collision_reward/meshes/custom_name
```


##### Add custom_file.STL in that folder 

##### In CMakeLists.txt at section "Install All Directories (Part 1)" add folder as following:

```bash
    meshes/custom_name
```

##### Output shuld be similar as this:
```bash
# Install All Directories (Part 1)
install(DIRECTORY
    urdf
    meshes
    meshes/human
    meshes/custom_name
    launch
```
