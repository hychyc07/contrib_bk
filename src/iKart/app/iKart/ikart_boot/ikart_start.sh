#!/bin/sh

sudo -u icub -i joystickCheck
ret1=$?

if [ $ret1 -eq 101 ]; then
    echo "joystick boot requested" 
    sudo -u icub -i yarp server &
    #sudo -u icub -i icub-cluster-server.sh start yarpserver3 &
    sleep 2
    sudo -u icub -i joystickCtrl --context joystickCtrl --from conf/ikart.ini --silent --force_configuration &
    sudo -u icub -i iCubInterface --context iKart --config conf/iKart.ini &
    sudo -u icub -i iKartCtrl --no_compass --joystick_connect &
elif [ $ret1 -eq 0 ]; then
    echo "No joystick boot requested" 
else
    echo "Unknown error" 
fi
