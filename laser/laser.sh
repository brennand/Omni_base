sudo chmod a+rw /dev/ttyACM0
rosparam set hokuyo_node/calibrate_time false
rosrun hokuyo_node hokuyo_node
rosrun rviz rviz -d `rospack find hokuyo_node`/hokuyo_test.vcg
