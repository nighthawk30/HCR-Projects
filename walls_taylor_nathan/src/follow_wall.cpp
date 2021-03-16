/*
  Nathan Taylor
  3/10/21
*/
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/String.h"
#include <string>
#include <map>
#include <cstdlib>
#include <cmath>

class Listen
{
public:
  Listen()
  {
    left_dist = 0;
    right_dist = 0;
    forward_dist = 0;
    back_dist = 0;
  }
  void poseCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  double left_dist;
  double forward_dist;
  double right_dist;
  double back_dist;
};

ros::Publisher pub;
ros::Subscriber sub;

void moveTurn(double distance, double ang_degrees);//ccw+
std::string setState(Listen listening);

void Listen::poseCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  //360 degrees 360 size
  forward_dist = scan->ranges[0];//forward
  left_dist = scan->ranges[90];//left
  back_dist = scan->ranges[180];
  right_dist = scan->ranges[270];
}

int main(int argc, char **argv)
{
  // INITIALIZE THE NODE
  ros::init(argc, argv, "wall_flower");
  ros::NodeHandle node;
  // Loop at 10Hz, publishing movement commands until we shut down
  ros::Rate rate(10);
  Listen listening;//create class instance in main to access callback
  pub = node.advertise<geometry_msgs::Pose2D>("/triton_lidar/vel_cmd", 10);
  sub = node.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &Listen::poseCallback, &listening);
  //init time
  ros::Duration(2.0).sleep();
  //
  ROS_INFO("Get RRRReeadddy toooooo Ruuummmmmmblllleee");
  //Setup
  moveTurn(5,0);
  moveTurn(0,-90);
  ROS_INFO("Setup Complete\n-------------------");
  //DEFINE STATE ACTION PAIRS
  std::map<std::string, double> qtable;//state, action(drive = .1, angle)
  qtable["L_c"] = -5;
  qtable["L_m"] = 0;
  qtable["L_f"] = 5;
  qtable["F_c"] = -10;
  double etime = ros::Time::now().toSec() + 30;
  std::string state = "";
  while (ros::Time::now().toSec() < etime)
    {
      //update state
      state = setState(listening);
      //choose action
      moveTurn(.1,qtable.at(state));
      ros::spinOnce();
    }
  ros::spin();
}

std::string setState(Listen listening)
{
  std::string state = "";
  if (listening.left_dist > .18)
    state = "L_f";
  else if (listening.left_dist < .18 && listening.left_dist > .16)
    state = "L_m";
  else if (listening.left_dist < .16)
    state = "L_c";
  if (listening.forward_dist < 1)
    state = "F_c";
  
  return state;
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
