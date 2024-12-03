# tue4tm00_group11 - Assignment 1

This project focuses on developing safe teleoperation and reactive navigation for the RoboCyl mobile robot using ROS2 in a Gazebo simulation. The teleoperation system adjusts user commands in real-time to ensure collision-free movement based on sensor data. Reactive navigation combines attractive and repulsive forces to autonomously guide the robot toward a goal while avoiding obstacles. Both solutions emphasize modularity, scalability, and safety, addressing key challenges in dynamic and unstructured environments.

## Running demos
Assuming the core_tue4tm00_humble package is installed (https://gitlab.tue.nl/core_robotics/courses/tue4tm00/core_tue4tm00_humble/-/blob/main/README.md?ref_type=heads):
- Create a ROS workspace
```
mkdir -p ~/tue4tm00_group11_ws/src
cd ~/tue4tm00_group11_ws
colcon build --symlink-install
```

- Clone this git repository. 
```
cd ~/tue4tm00_group11_ws/src
git clone git@gitlab.tue.nl:core_robotics/courses/tue4tm00/tue4tm00_groups_2024/tue4tm00_group11.git
```
- Install ROS dependencies using `rosdep`.
```
cd ~/tue4tm00_group11_ws
rosdep update
rosdep install --from-paths src -y --ignore-src
```

- Compile ROS workspace using `colcon`.
```
cd ~/tue4tm00_group11_ws
colcon build --symlink-install
```

- Source the core_tue4tm00_humble install folder.
```
source ~/tue4tm00_ws/install/local_setup.bash
```

 Source the tue4tm00_group11 install folder.
```
source ~/tue4tm00_group11_ws/install/local_setup.bash
```

- Start a demo launch file
```
ros2 launch demo_safe_twist_teleop_2D.launch.py
```
or
```
ros2 launch demo_safe_reactive_navigation.launch.py
```


