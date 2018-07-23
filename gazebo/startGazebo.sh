#!/bin/bash

export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-8/
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo/
export GAZEBO_MODEL_PATH=:$SMART_ROOT_ACE/repos/DataRepository/gazebo/models
#export GAZEBO_PLUGIN_PATH=~/.gazebo/
export GAZEBO_PLUGIN_PATH=$SMART_ROOT_ACE/lib
echo "GAZEBO_RESOURCE_PATH: $GAZEBO_RESOURCE_PATH"
echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"
if [ $# -eq 0 ]
  then
    echo "No world file as argument supplied !"
  else
    gazebo --verbose $1
fi

