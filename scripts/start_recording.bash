#!/usr/bin/env bash
source ~/.rosrc
rosnode kill /rosbag_play
roslaunch rustbot_bringup record_raw.launch < /dev/null > /dev/null 2> /dev/null & disown
roslaunch rustbot_bringup all.launch do_stereo:=false do_slam:=false online_stereo:=true do_gps:=true do_accumulation:=false do_fusion:=false do_zmq_publish:=false < /dev/null > /dev/null 2> /dev/null & disown
