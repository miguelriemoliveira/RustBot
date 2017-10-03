#!/usr/bin/env bash
source ~/.rosrc
roslaunch rustbot_bringup playback.launch < /dev/null > /dev/null 2> /dev/null & disown
roslaunch rustbot_bringup all.launch do_stereo:=true do_slam:=true online_stereo:=false do_gps:=false do_accumulation:=true do_fusion:=false do_zmq_publish:=true < /dev/null > /dev/null 2> /dev/null & disown
