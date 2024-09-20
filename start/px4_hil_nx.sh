#!/bin/bash

source ./install/setup.bash 

sudo -S chmod 777 ./src/adahil_model/lib/libsimodel_win64.so

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/src/adahil_model/lib

ros2 run adahil_model sim_model