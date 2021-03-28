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
#include "gazebo_msgs/GetModelState.h"
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
    c_state = std::vector<double>(360,0);//initialize array for scan
  }
  void poseCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    for (int i = 0; i < scan->ranges.size(); i++)//update saved scan
      c_state[i] = scan->ranges[i];
  }
  std::vector<double> c_state;//360 size array: distance at each degree
};

void moveTurn(double distance, double ang_degrees);//ccw+
std::vector<int> getState(Listen listening);
std::pair<double,double> getPosition();
double getDistance(std::pair<double,double> p1, std:: pair<double,double> p2);

ros::Publisher pub;
ros::Subscriber sub;
ros::ServiceClient client_set;
ros::ServiceClient client_get;

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
  client_get = node.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");

  //init time
  ros::Duration(2.0).sleep();
  ROS_INFO("SETUP COMPLETE ---------------------------");
  
  //INITIAL CONDITIONS
  int steps_followed = 0;//number of steps that the wall has been followed (each time reward is 0)
  float e_initial = .9;
  float d = .985;//reduce this so it trains longer/spends more time in random
  int episode_num = 0;
  std::vector<std::pair<double, double>> actions;
  actions.push_back(std::pair<double,double>(-45,.1));//hard left
  actions.push_back(std::pair<double,double>(-10,.1));//left
  actions.push_back(std::pair<double,double>(0,.1));//straight
  actions.push_back(std::pair<double,double>(10,.1));//right
  actions.push_back(std::pair<double,double>(45,.1));//hard right

  Q_table qt("learn");//initialize and read in qtable - learn mode

  //TRAINING LOOP - HERE WE GOOOOOO
  while (steps_followed < 1000)//success loop
    {
      steps_followed = 0;//reset success condition
      std::vector<std::pair<double,double>> history;//store the last three states
      std::vector<int> current_state;
      std::vector<int> past_state;
      float e_greedy = e_initial*pow(d, episode_num);//update e value
      bool fail = false;

      gazebo_msgs::SetModelState reset = restart.resetState();
      client_set.call(reset);//reset robot to semi-random pose+dir

      ROS_INFO("Episode: %i -----------------", episode_num);
      while (!fail)//episodes training (failed loop)
	{
	  ros::spinOnce();
	  //1. Choose Action based on current state (e-greedy)
	  past_state = getState(listening);
	  int act_index = qt.getAction(past_state, e_greedy);
	  ros::spinOnce();
	  
	  //2. Execute Action and observe state
	  std::pair<double, double> move = actions[act_index];
	  moveTurn(move.second, move.first);//I know I switched them, get over it
	  ros::spinOnce();
	  current_state = getState(listening);//observe state
	  
	  //3. Calculate Reward
	  int reward = qt.getReward(current_state);
	  if (past_state[2] == 1)//success condition update
	    {
	      steps_followed++;
	      if (steps_followed > 1000)
		  break;
	    }
	  else
	    steps_followed = 0;
	  
	  //DEBUG PRINT
	  ROS_INFO("LEFT: %i", past_state[2]);
	  ROS_INFO("FRONT LEFT %i", past_state[1]);
	  ROS_INFO("FRONT: %i", past_state[0]);
	  ROS_INFO("Action: %f", move.first);
	  ROS_INFO("Reward: %i", reward);
	  ROS_INFO("Steps Followed %i", steps_followed);

	  //4. Update Q-table
	  qt.updateTable(past_state, act_index, current_state);

	  //5. Check termination
	  std::pair<double,double> current_pose = getPosition();//get exact pose
	  history.push_back(current_pose);
	  if (history.size() > 3)
	    {
	      history.erase(history.begin());
	      int trapCount = 0;
	      for (int i = 0; i < history.size(); i++)
		if (getDistance(history[i], current_pose) < .01)//robot has not moved much
		  trapCount++;
	      
	      //the past three states are the same and bad, end it
	      if (trapCount == history.size() || current_pose.first == -5000)//specific flying bug
		{
		  fail = true;
		  ROS_INFO("ITS A TRAP!");
		}
	    }
	  ros::spinOnce();
	}
      qt.writeTable();//save updates
      episode_num++;
    }
  
  ROS_INFO("Simulation Complete: -------------------");
  ros::spin();
}

//return exact 2d pose of robot for training termination
std::pair<double,double> getPosition()
{
  gazebo_msgs::GetModelState getPose;
  getPose.request.model_name = "triton_lidar";
  client_get.call(getPose);
  std::pair<double,double> coordinates;
  coordinates.first = getPose.response.pose.position.x;
  coordinates.second = getPose.response.pose.position.y;
  if (getPose.response.pose.position.z > .1)
    {
      ROS_INFO("NO CAPES");//The robot will start flying sometimes when it hits a wall
      coordinates.first = -5000;//impossible value that is specifically checked above
    }
  return coordinates;
}

double getDistance(std::pair<double,double> p1, std:: pair<double,double> p2)
{
  return sqrt(pow((p1.first-p2.first),2)+pow(p1.second-p2.second,2));
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
      if (min_in_range[i] > .9)
	d_state[i] = 2;//far
      else if (min_in_range[i] <= .9 &&
	       min_in_range[i] > .6)
	d_state[i] = 1;//medium
      else
	d_state[i] = 0;//close
    }

  //Separate distance alotment for different angle ranges
  //LEFT
  if (min_in_range[2] > .6)
    d_state[2] = 2;//far
  else if (min_in_range[2] <= .6 &&
	   min_in_range[2] > .3)
    d_state[2] = 1;//medium
  else
    d_state[2] = 0;//close

  //Front left
  if (min_in_range[1] > .9)
    d_state[1] = 2;//far
  else if (min_in_range[1] <= .9 &&
	   min_in_range[1] > .4)
    d_state[1] = 1;//medium
  else
    d_state[1] = 0;//close

  //Front
  if (min_in_range[0] > .6)
    d_state[0] = 2;//far
  else if (min_in_range[0] <= .6 &&
	   min_in_range[0] > .4)
    d_state[0] = 1;//medium
  else
    d_state[0] = 0;//close

  
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
  geometry_msgs::Pose2D stop;//d
 
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
