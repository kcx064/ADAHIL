#!/bin/bash

# 获取执行该脚本时的路径，并赋值给变量 RUN_PATH
RUN_PATH=$(pwd)

# 打印当前路径
echo "当前路径是: $RUN_PATH"

sudo -S chmod 777 ./src/adahil_model/lib/libsimodel_win64.so

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/src/adahil_model/lib

source ./install/setup.bash 

ros2 launch adahil_model run_sim_model.py & PID1=$!
sleep 5s

ros2 launch adahil_sensor_send run_sensor_send.py & PID2=$!
sleep 5s

ros2 launch adahil_send_unrealdata run_unrealdata_send.py & PID3=$!
sleep 5s

ros2 launch adahil_gps_send run_gps_send.py & PID4=$!
sleep 5s

ros2 launch adahil_pwm_read run_pwm_read.py & PID5=$!

wait
kill -9 PID1 & 
kill -9 PID2 & 
kill -9 PID3 & 
kill -9 PID4 & 
kill -9 PID5 & 
exit