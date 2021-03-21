/*
  Nathan Taylor
  3/17/21
*/
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"
#include "gazebo_msgs/SetModelState.h"
#include "std_msgs/String.h"
#include "tf2/LinearMath/Quaternion.h"
#include "qtable.h"
#include <string>
#include <map>
#include <cstdlib>
#include <cmath>
#include <vector>

class Listen//callback class
{
public:
  Listen()
  {
    //relative directions
    c_state = {0,0,0,0,0};//continuous state
  }
  void poseCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  std::vector<double> c_state;
};

//callback method
void Listen::poseCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  //360 degrees 360 size array
  //ROS_INFO("Left: %f", left_dist);//comment out
  c_state[0] = scan->ranges[270];//left
  c_state[1] = scan->ranges[315];
  c_state[2] = scan->ranges[0];//forward
  c_state[3] = scan->ranges[45];
  c_state[4] = scan->ranges[90];//right
  //Debugging
  /*
  ROS_INFO("Left: %f", scan->ranges[270]);
  ROS_INFO("1:30: %f", scan->ranges[315]);
  ROS_INFO("Forward: %f", scan->ranges[0]);
  ROS_INFO("10:30: %f", scan->ranges[45]);
  ROS_INFO("Right: %f", scan->ranges[90]);
  ROS_INFO("--------------------------");
  */
}

void moveTurn(double distance, double ang_degrees);//ccw+
std::vector<int> getState(Listen listening);

ros::Publisher pub;
ros::Subscriber sub;
ros::ServiceClient client;

int main(int argc, char **argv)
{
  // INITIALIZE THE NODE
  ros::init(argc, argv, "wall_flower");
  ros::NodeHandle node;
  ros::Rate rate(10);
  
  //Connection Setup
  Listen listening;//create class instance in main to access callback
  pub = node.advertise<geometry_msgs::Pose2D>("/triton_lidar/vel_cmd", 10);
  sub = node.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &Listen::poseCallback, &listening);
  client = node.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  //init time
  ros::Duration(2.0).sleep();
  
  //Setup - change to move to a random position on the map
  //maybe only a selection of a set of positions
  gazebo_msgs::SetModelState reset;
  reset.request.model_state.model_name = "triton_lidar";
  reset.request.model_state.pose.position.x = 3.7;
  client.call(reset);
  //moveTurn(0,-90);
  ROS_INFO("Setup Complete: -------------------");

  //TESTING CODE

  /*
1. Choose action based on current state (e-greedy)
2. Execute action and observe state
3. Calculate reward
4. Update Q-table: with current reward + highest reward of next possible action state pair
5. Check termination conditions
   */

  //TESTING CODE
  Q_table qt;
  //ROS_INFO("Size: %i", qt.qsa.size());
  qt.writeTable();
  
  ROS_INFO("Simulation Complete: -------------------");
  //TESTING CODE

  
  //Run
  /*
  double etime = ros::Time::now().toSec() + 30;
  while (ros::Time::now().toSec() < etime)
    {
      //choose action
      moveTurn(.1,getAction(qt, getState(listening)));
      ros::spinOnce();
    }
  */
  ros::spin();
}
//define states and return current state
std::vector<int> getState(Listen listening)
{
  std::vector<int> d_state(5, 0);//discrete state
  for (int i = 0; i < d_state.size(); i++)
    {
      if (listening.c_state[i] > .18)
	{
	  d_state[i] = 2;//far
	}
      else if (listening.c_state[i] < .18 &&
	       listening.c_state[i] > .16)
	{
	  d_state[i] = 1;//medium
	}
      else// < .16
	{
	  d_state[i] = 0;//close
	}
    }  
  return d_state;
}

//need to be able to do both at once
void moveTurn(double distance, double ang_degrees)
{
  double angular_speed = .5;
  if (ang_degrees <= 0)
    angular_speed *= -1;
  double speed = .2;
  double PI = 3.141592653589693;
  double ang_rad = ang_degrees * PI/180;
  double cdist = 0;//current distance
  double cang = 0;//current angle
  geometry_msgs::Pose2D stop;

  double start = ros::Time::now().toSec();
  while (abs(cang) < abs(ang_rad) || cdist < distance)
    {
      cang = angular_speed * (ros::Time::now().toSec() - start);
      cdist = speed * (ros::Time::now().toSec() - start);
      //set msg
      geometry_msgs::Pose2D msg;
      if (abs(cang) < abs(ang_rad))
	msg.theta = angular_speed;
      if (cdist < distance)
	msg.x = speed;
      //print message
      pub.publish(msg);
      //give up
      ros::spinOnce();
    }
  pub.publish(stop);
}
