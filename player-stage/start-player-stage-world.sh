#!/bin/bash


PLAYER_BIN="player"
PLAYER_DIR="$SMART_ROOT_ACE/repos/DataRepository/player-stage"

world=$1

if [ "$world" = "" ]; then

	echo -n "
Please choose a world to load:

(0) Hospital World

(1) Racetrack World (SRRC Laboratory Scale)
(2) Cylinder World (SRRC Laboratory Scale)
(3) Corner World (SRRC Laboratory Scale)
(4) Wallfollower World (SRRC Laboratory Scale)
(5) Wallfollower World 2 (SRRC Laboratory Scale)

(6) Simple World

> "

	read world

fi

case "$world" in
        0)
                playercfg="smart_hospital.cfg"
                ;;

        1)
                playercfg="smart_racetrack.cfg"
                ;;

        2)
                playercfg="smart_cylinder.cfg"
                ;;

        3)
                playercfg="smart_cornermap.cfg"
                ;;

        4)
                playercfg="smart_wallfollower.cfg"
                ;;

        5)
                playercfg="smart_wallfollower2.cfg"
                ;;

        6)
                playercfg="simple.cfg"
                ;;

        *)
                echo "Unknown World: '$world'"
                exit
                ;;
esac

DIR=`pwd`

echo
echo "Starting world $playercfg ..."
echo 

cd $PLAYER_DIR
xterm -title "Player Stage Simulator Terminal" -e "pwd; $PLAYER_BIN $playercfg || bash; sleep 2" &
cd $DIR
