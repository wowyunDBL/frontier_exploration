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

rosrun mapping_exploretrunk_info_node.py
roslaunch depth_app depth2ls.launch