#!/bin/bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/pi/ADAHIL/src/adahil_model/lib
source ./install/setup.bash 
# ros2 run adahil_model sim_model
ros2 launch adahil_model run_sim_model.py & PID1=$!
sleep 5s

ros2 launch adahil_sensor_send run_sensor_send.py & PID2=$!

wait
kill -9 PID1 PID2 & 
exit