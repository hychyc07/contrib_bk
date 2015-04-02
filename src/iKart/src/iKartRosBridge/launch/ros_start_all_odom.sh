roscore &
sleep 3
iKartRosBridge & 
sleep 3
$IKART_ROS_BRIDGE/launch/load_map.sh $IKART_ROS_BRIDGE/../../../iKart/app/iKart/maps/rbcsLab2 &
sleep 3
roslaunch $IKART_ROS_BRIDGE/launch/ikart_localize.launch &
sleep 3
roslaunch $IKART_ROS_BRIDGE/launch/rviz_navigate.launch &
