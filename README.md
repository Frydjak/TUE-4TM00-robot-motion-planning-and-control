# 4TM00 Robot Motion Planning and Control - group 11

This project focuses on developing safe teleoperation and reactive navigation for the RoboCyl mobile robot using ROS2 in a Gazebo simulation. The teleoperation system adjusts user commands in real-time to ensure collision-free movement based on sensor data. Reactive navigation combines attractive and repulsive forces to autonomously guide the robot toward a goal while avoiding obstacles. Both solutions emphasize modularity, scalability, and safety, addressing key challenges in dynamic and unstructured environments.

## Results
### Final Project - Safe Robot Navigation under Intermitted Localization
https://github.com/user-attachments/assets/263ec813-00c4-4d6c-95b6-e36b9bb39206

### Assignment 2 - Search-Based Path Planning & Safe Path Following
[Assignment2_Demo.webm](https://github.com/user-attachments/assets/a2dc088c-021b-4586-ab38-c91baaae933a)

### Assignment 1 - Safe Teleoperation & Reactive Robot Navigation
[Assignment1_Demo.webm](https://github.com/user-attachments/assets/6310e29d-a6df-4c65-96f4-7ce2e25eccfd)

[Assignment1_PotentialFields.webm](https://github.com/user-attachments/assets/58afd71c-8f55-4e8c-8ac0-ee65bc1a4868)


## Running demos
Assuming the core_tue4tm00_humble package is installed(https://gitlab.tue.nl/core_robotics/courses/tue4tm00/core_tue4tm00_humble/-/blob/main/README.md?ref_type=heads):
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

- Source the tue4tm00_group11 install folder.
```
source ~/tue4tm00_group11_ws/install/local_setup.bash
```

- Start a demo launch file.

for assignment 1:
```
ros2 launch group11_tue4tm00_assignment1 demo_safe_twist_teleop_2D.launch.py
```
or
```
ros2 launch group11_tue4tm00_assignment1 demo_safe_reactive_navigation.launch.py
```

for assignment 2:
```
ros2 launch group11_tue4tm00_assignment2 demo_safe_navigation_costmap.launch.py
```
or
```
ros2 launch group11_tue4tm00_assignment2 demo_safe_navigation_costmap.launch.py
```
or
```
ros2 launch group11_tue4tm00_assignment2 demo_safe_navigation_costmap.launch.py
```
For smooth path following performance it is recommended to generate path only once, at the beginning of simulation. This can easily be done by modifying `min_move_threshold` value inside `search_based_path_planner.yaml` file to large value, for example: 
```
min_move_threshold: 100
```
This will ensure no path updates happen during simulation.
