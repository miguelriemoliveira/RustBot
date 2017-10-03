#!/usr/bin/env bash
source ~/.rosrc
rosnode kill /imu_republish_to_enu
rosnode kill /rosbag_record 
rosnode kill /sev_publisher
rosnode kill /stereo/left
rosnode kill /stereo/right
