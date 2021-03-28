Nathan Taylor
3/27/2021
QLearning for wall following Triton Robot

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
   
5. Launch the simulation in training mode
roslaunch walls_taylor_nathan wall_following_learn.launch

6. Launch the simulation in demo mode to use a learned policy
roslaunch walls_taylor_nathan wall_following_demo.launch

Notes:
-The training mode node saves its qtable to a file called tablesave.txt after each episode
-If there is a pre-existing tablesave.txt, the qtable will initialize with those values instead, this means that you can end and restart the simulation without losing information.
-You can manually change the episode_num in the wall_qlearn.cpp to choose where to start
-Training mode has debug statements about the state action and reward
-If the robot gets stuck it starts a new episode and resets to a random one of predetermined positions with a random orientation
-Sometimes the robot starts flying after running into a wall, it also starts a new episode after this
-This package does not include a tablesave.txt, but it does include a fully trained tablepolicy.txt, if you wish to use this trained qtable in training mode, copy it to tablesave.txt

-The policy mode node: wall_qpolicy.cpp intializes its qtable from a file called tablepolicy.txt. In order to use a trained qtable in policy mode, you must manually copy it to tablepolicy.txt. Policy mode
-Demo mode has no debug statements and resets to a random position after a given number of steps

-To train this qtable, it took about 3.5 hours or ~70 episodes
