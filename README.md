# Follow-Everything

# Overview
Welcome to the [**follow everying**](https://follow-everything.github.io/) code repository, a robust leader-following framework to enable the robot to follow a leader with arbitrary shapes.

[![](https://github.com/user-attachments/assets/b3417149-7720-4d2d-b7d9-3071adb6462f)](https://www.youtube.com/watch?v=N3d6XKs4Jns)


![Ubuntu 22.04 Test Passed](https://img.shields.io/badge/Ubuntu-22.04-blue)

This code has been tested on ubuntu22.04 with ros2 humble and ros1 noetic. The code with the Docker environment is being processed and will be released recently.

# Description for files

* go2_ws: simulator of unitree go2, sim2 segmentation (in ros2)
* pedsim_ws: motion planning algorithm (in ros1)
* ros1_bridge: map the topics between ros1 and ros2


# How to run
##### 1. Simulator:
```bash
source ~/Documents/go2_ws/install/setup.bash
ros2 launch go2_config env_forest.launch.py rviz:=true
```


##### 2. Move_base (following algorithm)
```bash
source /opt/ros/noetic/setup.bash
source ~/Documents/pedsim_ws/devel/setup.bash
roslaunch move_base move_base.launch
```

##### 3. Ros1 bridge
```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash
source ~/Documents/ros1_bridge/install/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
```

##### 4. Sam model:
```bash
source ~/Documents/go2_ws/install/setup.bash
conda activate sam_go2
ros2 launch tracking gazebo_tracking.launch.py
```





# How to build
##### 1. Go2_ws
```bash
source /opt/ros/humble/setup.bash 
cd ~/Documents/go2_ws && colcon build --symlink-install --packages-ignore tracking
conda activate sam_go2
cd ~/Documents/go2_ws && python -m colcon build --symlink-install --packages-select tracking
```
##### 2. Pedsim_ws
```bash
source /opt/ros/noetic/setup.bash 
cd ~/Documents/pedsim_ws && catkin_make
```

##### 3. Ros1_bridge
```bash
source /opt/ros/humble/setup.bash
cd Documents/ros1_bridge/ && colcon build --symlink-install --packages-skip ros1_bridge
source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash
cd Documents/ros1_bridge/ && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

