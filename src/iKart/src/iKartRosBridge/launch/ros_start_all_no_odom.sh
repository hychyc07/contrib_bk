roscore &
sleep 3
iKartRosBridge --no_odom_tf &
sleep 3
$IKART_ROS_BRIDGE/launch/load_map.sh $IKART_ROS_BRIDGE/../../../iKart/app/iKart/maps/rbcsLab2 &
sleep 3
roslaunch $IKART_ROS_BRIDGE/launch/ikart_laseronly_localize.launch &
sleep 3
roslaunch $IKART_ROS_BRIDGE/launch/rviz_navigate.launch &
