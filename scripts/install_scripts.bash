#!/usr/bin/env bash
DIR=`pwd`
echo $DIR

#To be called by windows
ln -si $DIR/.rosrc $HOME/.rosrc
ln -si $DIR/start_recording.bash $HOME/start_recording.bash
ln -si $DIR/stop_recording.bash $HOME/stop_recording.bash

#To be called by crontab
ln -si $DIR/bringup_system.bash $HOME/bringup_system.bash

