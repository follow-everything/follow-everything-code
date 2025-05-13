# follow-everything-code

The code with the Docker environment is being processed and will be released at the end of May.


# How to run
##### 1. Start worlds:
```bash
source ~/Documents/go2_ws/install/setup.bash
ros2 launch go2_config env_forest.launch.py rviz:=true
```


##### 2. Move_base
```bash
source /opt/ros/noetic/setup.bash
source ~/Documents/pedsim_ws/devel/setup.bash
roslaunch move_base move_base.launch
```

##### 3. ros1 bridge
```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash
source ~/Documents/ros1_bridge/install/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
```

##### 4. Start sam:
```bash
source ~/Documents/go2_ws/install/setup.bash
conda activate sam_go2
ros2 launch tracking gazebo_tracking.launch.py
```





# How to build
##### 1. go2_ws
```bash
source /opt/ros/humble/setup.bash 
cd ~/Documents/go2_ws && colcon build --symlink-install --packages-ignore tracking
conda activate sam_go2
cd ~/Documents/go2_ws && python -m colcon build --symlink-install --packages-select tracking
```
##### 2. ros1_bridge
```bash
source /opt/ros/humble/setup.bash
cd Documents/ros1_bridge/ && colcon build --symlink-install --packages-skip ros1_bridge
source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash
cd Documents/ros1_bridge/ && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

