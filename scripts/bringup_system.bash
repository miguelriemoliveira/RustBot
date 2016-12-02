#!/usr/bin/env bash
source ~/.rosrc
roslaunch rustbot_bringup all.launch do_stereo:=true online_stereo:=true respawn:=true & disown

