#!/bin/bash
gnome-terminal -- bash -c "roscore"
sleep 0.5
gnome-terminal -t --title="realsense2_camera" -- bash -c "roslaunch realsense2_camera point.launch"
sleep 0.5
gnome-terminal -t --title="point_cloud_process" -- bash -c "roslaunch point_cloud_process get_table_top_points.launch"
sleep 0.5
gnome-terminal -t --title="gpd_ros" -- bash -c "roslaunch gpd_ros tinyarm.launch"
sleep 0.5
gnome-terminal -t --title="robot_sim" -- bash -c "roslaunch robot_sim gpd_run.launch "
#gnome-terminal -t --title="robot_sim" -- bash -c "roslaunch robot_sim geo_run.launch "
sleep 0.5
gnome-terminal -t --title="rviz" -- bash -c "rviz"

#ros_eigen_params