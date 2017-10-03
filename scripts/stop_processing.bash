#!/usr/bin/env bash
source ~/.rosrc
rosnode kill /Gimbal_baselink 
rosnode kill /accumulatepointcloud 
rosnode kill /base_link_to_gps
rosnode kill /base_link_to_imu
rosnode kill /camera_to_gimbal
rosnode kill /imu_republish_to_enu
rosnode kill /left_optical2stereo_camera
rosnode kill /map_odom_tf
rosnode kill /pcl_manager
rosnode kill /republish_left
rosnode kill /republish_left_color
rosnode kill /republish_right
rosnode kill /republish_right_color
rosnode kill /rosbag_play
rosnode kill /sev_publisher
rosnode kill /stereo/stereo_image_proc 
rosnode kill /stereo_odometer 
rosnode kill /voxel_grid
