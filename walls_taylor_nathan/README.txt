cd ~/catkin_ws
. ~/catkin_ws/devel/setup.bash
catkin_make
source ~/Stingray-Simulation/catkin_ws/devel/setup.bash
source ~/Stingray-Simulation/stingray_setup.bash

roslaunch walls_taylor_nathan wall_following_v1.launch
