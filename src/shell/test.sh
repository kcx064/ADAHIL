export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/pi/ADAHIL/src/adahil_model/lib
. install/setup.bash 
# ros2 run adahil_model sim_model
ros2 launch adahil_model run_sim_model.py