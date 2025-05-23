# 4TM00 Robot Motion Planning and Control
This class focused on developing autonomous navigation capabilities for mobile robots using ROS2 and Gazebo simulation. The work is divided into three stages: safe teleoperation, reactive navigation, and multi-goal path planning in complex environments. Each phase progressively integrates more advanced planning, control, and safety mechanisms. Key goals include ensuring collision-free motion, modularity of software, and robustness to localization challenges. Below are short summaries and demo results of each milestone.

## Results
### Group Project - Safe Robot Navigation under Intermitted Localization
Developed a navigation system for a mobile robot with differential drive in a known environment with low-frequency global localization. Combined odometry, costmap generation, A* planning, pure-pursuit path following, and multi-goal management for safe and sequential navigation.

https://github.com/user-attachments/assets/263ec813-00c4-4d6c-95b6-e36b9bb39206

### Assignment 2 - Search-Based Path Planning & Safe Path Following
Implemented a minimum-cost navigation system using a costmap derived from an occupancy grid. Path planning used the A* algorithm, and path following employed a proportional controller for safe execution.

[Assignment2_Demo.webm](https://github.com/user-attachments/assets/a2dc088c-021b-4586-ab38-c91baaae933a)

### Assignment 1 - Safe Teleoperation & Reactive Robot Navigation
Designed a safty layer for manual teleoperation that modifies robots velocity to avoid collisions. Based on laser scan data a potential field is generated which aims to "push" robot away from obstacles.

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

for assignment 2:
```
ros2 launch group11_tue4tm00_assignment2 demo_sensor_based_path_follower.launch.py
```

for Group Project:
```
ros2 launch group11_tue4tm00_project demo_navigation.launch.py
```

For smooth path following performance it is recommended to generate path only once, at the beginning of simulation. This can easily be done by modifying `min_move_threshold` value inside `search_based_path_planner.yaml` file to large value, for example: 
```
min_move_threshold: 100
```
This will ensure no path updates happen during simulation.

This project was completed in collaboration with one other team member.
