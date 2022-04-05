frontier_exploration ![asdf](https://travis-ci.org/paulbovbel/frontier_exploration.svg?branch=hydro-devel)
====================



Implementation of frontier exploration (http://www.robotfrontier.com/papers/cira97.pdf) for ROS, extending on the existing navigation stack (costmap_2d, move_base).

Wiki: http://wiki.ros.org/frontier_exploration

API Doc: http://docs.ros.org/hydro/api/frontier_exploration/html/annotated.html

Video:

[![Demo Video](http://img.youtube.com/vi/3W1ufJ7rpCA/0.jpg)](https://www.youtube.com/watch?v=3W1ufJ7rpCA)

roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
rosparam set /apf/trigger True
rosrun frontier_exploration potential_field_v1.py
rosrun frontier_exploration plot_node.py

roslaunch depth_app depth2ls_simulation.launch
roslaunch depth_app hector_mower_simulation.launch

rosbag record /cmd_vel /explore/traj /explore/potential/af /explore/potential/rf /explore/trajRTheta /odom /tree/trunk_info /explore/state /scan /scan_filtered

rosbag record /camera/depth/image_raw/coressed /camera/depth/camera_info /odom /cmd_vel /scan /scan_filtered /tree/trunk_info
rosrun mapping_explorer trunk_info_node.py
roslaunch mapping_explorer depth2ls_gazebo.launch
rosrun mapping_explorer plot_scan.py

## 0405
```
rosbag record /husky_velocity_controller/cmd_vel /explore/traj /explore/potential/af /explore/potential/rf /explore/trajRTheta /outdoor_waypoint_nav/odometry/filtered_map /tree/trunk_info /explore/state /scan /scan_filtered

rosrun mapping_explorer trunk_info_node.py
roslaunch realsense2_camera hector_mower.launch
rosrun map_server map_saver --occ 11 --free 10 -f /home/ncslaber/reso4