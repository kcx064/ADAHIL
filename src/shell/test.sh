#!/bin/bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/pi/ADAHIL/src/adahil_model/lib
source ./install/setup.bash 
# ros2 run adahil_model sim_model
ros2 launch adahil_model run_sim_model.py & PID1=$!
sleep 5s

ros2 launch adahil_sensor_send run_sensor_send.py & PID2=$!
sleep 5s

ros2 launch adahil_send_unrealdata run_unrealdata_send.py & PID3=$!
sleep 5s

ros2 launch adahil_gps_send run_gps_send.py & PID4=$!
sleep 5s

# ros2 launch example_cpp run_topic_pub.py & PID4=$!

wait
kill -9 PID1 & 
kill -9 PID2 & 
kill -9 PID3 & 
kill -9 PID4 & 
exit