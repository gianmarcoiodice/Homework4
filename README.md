# Robotics Lab Homework 4

### 1. Constructing a Gazebo World and Spawning the Mobile Robot
- **Run:** 
in a first terminal : 
```
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py 
```
in a second terminal :
```
ros2 run rqt_image_view rqt_image_view
```
### 2. Setting Up Static TF Goals and Navigation
- **Run:** 
in a first terminal : 
```
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py 
```
in a second terminal :
```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
```
in a third terminal :
```
ros2 run rl_fra2mo_description follow_waypoints_goals.py
```
### 3. Map the environment
-**Run:**
in a first terminal:
```
$ ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```
in a second terminal:
```
$ ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
```
in a third terminal:
```
$ ros2 run rl_fra2mo_description follow_waypoints.py
```
To save the pose:
```
$ cd src
$ ros2 bag record -o subset /pose
```
```
$ ros2 launch rl_fra2mo_description fra2mo_slam.launch.py
```
To save the map
```
$ cd src
$ ros2 run nav2_map_server map_saver_cli -f map
```

### 4. Vision-Based Navigation
- **Run:** 
in a first terminal: 
```
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py 
```
in a second terminal: 
```
ros2 run rl_fra2mo_description static_world_broadcaster.py 
```
in a third terminal:
```
ros2 launch rl_fra2mo_description merge.launch.py 
```
in a fourth terminal:
```
ros2 run rl_fra2mo_description ARUCO_waypoints.py 
```
in order to visualize the tf publication run in a terminal: 
```
ros2 run tf2_ros tf2_echo world aruco_marker 
```

Visualize on Rviz
```
ros2 launch rl_fra2mo_description display_fra2mo.launch.py
```
