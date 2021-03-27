/*
  Nathan Taylor  
3/24/21
Training Program for QLearning: Robot follows a wall
*/

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"
#include "gazebo_msgs/SetModelState.h"
#include "std_msgs/String.h"
#include "qtable.h"
#include "setstate.h"

#include <string>
#include <map>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <ctime>

class Listen//callback class
{
public:
  Listen()
  {
    c_state = std::vector<double>(360,0);
  }
  void poseCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    for (int i = 0; i < scan->ranges.size(); i++) 
      c_state[i] = scan->ranges[i];
  }
  std::vector<double> c_state;//360 size array: distance at each degree
};

void moveTurn(double distance, double ang_degrees);//ccw+
std::vector<int> getState(Listen listening);

ros::Publisher pub;
ros::Subscriber sub;
ros::ServiceClient client_set;

int main(int argc, char **argv)
{
  // INITIALIZE THE NODE
  ros::init(argc, argv, "wall_flower");
  ros::NodeHandle node;
  ros::Rate rate(10);
  srand(time(NULL));
  
  //Connection Setup
  Listen listening;//create class instance in main to access callback
  SetState restart;
  pub = node.advertise<geometry_msgs::Pose2D>("/triton_lidar/vel_cmd", 10);
  sub = node.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &Listen::poseCallback, &listening);
  client_set = node.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  //init time
  ros::Duration(2.0).sleep();
  ROS_INFO("SETUP COMPLETE ---------------------------");
  
  //INITIAL CONDITIONS
  std::vector<std::pair<double, double>> actions;
  actions.push_back(std::pair<double,double>(-45,.1));//hard left
  //actions.push_back(std::pair<double,double>(-10,.1));//left
  actions.push_back(std::pair<double,double>(0,.1));//straight
  //actions.push_back(std::pair<double,double>(10,.1));//right
  actions.push_back(std::pair<double,double>(45,.1));//hard right

  Q_table qt("demo");//initialize and read in qtable
  float e_policy = -1;//always choose based on policy
  int starts = 0;
 
  //TRAINING LOOP - HERE WE GOOOOOO
  while (true)//success loop
    {
      std::vector<int> current_state;
      int proof_steps = 0;
      gazebo_msgs::SetModelState reset = restart.demoState(starts);
      client_set.call(reset);//reset robot to semi-random pose+dir

      while (proof_steps < 100)//episodes training (failed loop)
	{
	  ros::spinOnce();
	  //1. Choose Action based on current state (e-greedy)
	  current_state = getState(listening);
	  int act_index = qt.getAction(current_state, e_policy);
	  ros::spinOnce();
	  
	  //2. Execute Action and observe state
	  std::pair<double, double> move = actions[act_index];
	  moveTurn(move.second, move.first);//I know I switched them, get over it
	  ros::spinOnce();
	  proof_steps++;
	}
      starts++;
    }
  
  ROS_INFO("Simulation Complete: -------------------");
  ros::spin();
}

//define states and return current state
std::vector<int> getState(Listen listening)
{
  std::vector<double> min_in_range(3, 0);//minimum distance for each angle range
  std::vector<int> d_state(3,0);//discrete state set
  std::vector<std::pair<int,int>> aranges;
  aranges.push_back(std::pair<int,int>(338,382));//front angle range: 45 degrees
  aranges.push_back(std::pair<int,int>(23,67));//left forward angle range %360 for actual angle
  aranges.push_back(std::pair<int,int>(68,112));//left angle range

  //for each element in d_state
  for (int i = 0; i < min_in_range.size(); i++)
    {
      min_in_range[i] = listening.c_state[aranges[i].first % 360];//set initial value to first in range
      for (int j = aranges[i].first; j < aranges[i].second; j++)//look through all angles in range 
	if (min_in_range[i] > listening.c_state[j % 360])//find lowest
	  min_in_range[i] = listening.c_state[j % 360];
    }
  
  //Determine discrete state based on minimum distance of continuous state
  for (int i = 0; i < d_state.size(); i++)
    {
      if (min_in_range[i] > .4)
	  d_state[i] = 2;//far
      else if (min_in_range[i] <= .4 &&
	       min_in_range[i] > .25)
	  d_state[i] = 1;//medium
      else// < .16
	  d_state[i] = 0;//close
    }  
  return d_state;
}

//need to be able to do both at once
void moveTurn(double distance, double ang_degrees)
{
  double angular_speed = 2.3;
  if (ang_degrees <= 0)
    angular_speed *= -1;
  double linear_speed = .3;
  double PI = 3.141592653589693;
  double ang_rad = ang_degrees * PI/180;
  double cdist = 0;//current distance
  double cang = 0;//current angle
  geometry_msgs::Pose2D stop;

  double start = ros::Time::now().toSec();
  while (abs(cang) < abs(ang_rad) || cdist < distance)
    {
      cang = angular_speed * (ros::Time::now().toSec() - start);
      cdist = linear_speed * (ros::Time::now().toSec() - start);
      //set msg
      geometry_msgs::Pose2D msg;
      if (abs(cang) < abs(ang_rad))
	msg.theta = angular_speed;
      if (cdist < distance)
	msg.x = linear_speed;
      //print message
      pub.publish(msg);
      //give up
      ros::spinOnce();
    }
  pub.publish(stop);
}

  //EXTENSIVE TESTING
  /*
0 Forward
45 Front Left
90 left
135 back left
180 back
225 back right
270 right
315 front right
*/
