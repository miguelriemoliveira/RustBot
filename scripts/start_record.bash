#!/usr/bin/env bash
source ~/.rosrc
roslaunch rustbot_bringup record_raw.launch < /dev/null > /dev/null 2> /dev/null & disown
