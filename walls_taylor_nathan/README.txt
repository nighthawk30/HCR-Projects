Nathan Taylor
3/11/2021
Basic Wall Following Triton Robot

Note: You must have previously cloned the Stingray Repository:
https://gitlab.com/HCRLab/stingray-robotics/Stingray-Simulation

Steps:
1. Clone the repository or extract the package into your workspace source files (catkin_ws/src)

2. Move to the top of your workspace (catkin_ws) and add the package:
. ~/catkin_ws/devel/setup.bash

3. Build the package
catkin_make

4. Source the stingray simulation:
source ~/Stingray-Simulation/catkin_ws/devel/setup.bash
source ~/Stingray-Simulation/stingray_setup.bash
   
5. Launch the simulation
roslaunch walls_taylor_nathan wall_following_v1.launch
