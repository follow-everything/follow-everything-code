# Message that contains intermediate results 
# and diagnostics of the (predictive) planner.

std_msgs/Header header

# The planned trajectory (or if multiple plans exist, all of them)
teb_local_planner/TrajectoryMsg[] trajectories

# Index of the trajectory in 'trajectories' that is selected currently
uint16 selected_trajectory_idx

# List of active obstacles
teb_local_planner/ObstacleArrayMsg obstacles_msg


