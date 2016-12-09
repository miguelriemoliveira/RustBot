#!/usr/bin/env bash
source ~/.bashrc
source ~/.rosrc

#Launch the system as described in https://github.com/miguelriemoliveira/RustBot#usage
#roslaunch rustbot_bringup all.launch fps:=10 do_stereo:=true online_stereo:=true

touch /tmp/executed_bringup_system.bash.log

roscore

