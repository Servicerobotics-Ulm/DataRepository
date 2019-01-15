#!/bin/bash

FROM=$(pwd)
cd $SMART_ROOT_ACE/repos/DataRepository/gazebo

export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-8/
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo/
export GAZEBO_MODEL_PATH=:$SMART_ROOT_ACE/repos/DataRepository/gazebo/models
#export GAZEBO_PLUGIN_PATH=~/.gazebo/
#export GAZEBO_PLUGIN_PATH=$SMART_ROOT_ACE/lib
export GAZEBO_PLUGIN_PATH=~/.gazebo/plugins
echo "GAZEBO_RESOURCE_PATH: $GAZEBO_RESOURCE_PATH"
echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"


if [ $# -eq 0 ]
  then
    	echo -n "
	Please choose a world to load:

	(0) Empty
	(1) Office

	> "

	read WORLD

	case "$WORLD" in
		0)
		        echo "Selected empty world"
		        WORLD=0
		        ;;
		1)
		        echo "Selected office world"
		        WORLD=1
		        ;;
		*)
		        echo "Unknown World: '$WORLD'"
		        exit
		        ;;
	esac

	echo -n "
	Please choose a robot to load:

	(0) Pioneer3DX
	(1) Tiago
	(2) Robotino3

	> "

	read ROBOT

	case "$ROBOT" in
		0)
		        echo "Selected Pioneer3DX"
		        if [ $WORLD -eq 0 ]; then
		        FILE="worlds/pioneerEmpty.world"
		        elif [ $WORLD -eq 1 ]; then
		        FILE="worlds/pioneerOffice.world"
		        fi
		        ;;
		1)
		        echo "Selected Tiago"
		        if [ $WORLD -eq 0 ]; then
		        FILE="worlds/tiagoEmpty.world"
		        elif [ $WORLD -eq 1 ]; then
		        FILE="worlds/tiagoOffice.world"
		        fi
		        ;;
		2)
		        echo "Selected Robotino3"
		        if [ $WORLD -eq 0 ]; then
		        FILE="worlds/robotino3-laserEmpty.world"
		        elif [ $WORLD -eq 1 ]; then
		        FILE="worlds/robotino3-laserOffice.world"
		        fi
		        ;;
		*)
		        echo "Unknown Robot: '$ROBOT'"
		        exit
		        ;;
	esac
	xterm -title "Gazebo Terminal" -e "pwd; gazebo --verbose $FILE || bash; sleep 5" &
  else
    xterm -title "Gazebo Terminal" -e "pwd; gazebo --verbose $1 || bash; sleep 5" &


fi

cd $FROM
