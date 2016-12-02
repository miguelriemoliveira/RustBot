#!/usr/bin/env bash
DIR=`pwd`
echo $DIR

ln -si $DIR/rosrc $HOME/rosrc
ln -si $DIR/rosrc $HOME/start_recording.bash
ln -si $DIR/rosrc $HOME/stop_recording.bash

