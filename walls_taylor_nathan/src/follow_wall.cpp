/*
  Nathan Taylor
  3/10/21
*/
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include <cstdlib>

#include <cmath>
ros::Publisher pub;
ros::Subscriber sub;

void move(double distance);
void turn(double angle);//ccw

void poseCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  ROS_INFO("Min=: [%f]",scan->ranges[scan->angle_min]);
  ROS_INFO("Half=: [%f]",scan->ranges[scan->angle_max/2]);
  ROS_INFO("Max=: [%f]",scan->ranges[scan->angle_max]);
  ROS_INFO("---------------------------");
}

int main(int argc, char **argv)
{
  // INITIALIZE THE NODE
  ros::init(argc, argv, "wall_flower");
  ros::NodeHandle node;
  // Loop at 10Hz, publishing movement commands until we shut down
  ros::Rate rate(10);
  pub = node.advertise<geometry_msgs::Pose2D>("/triton_lidar/vel_cmd", 10);
  sub = node.subscribe<sensor_msgs::LaserScan>("/scan", 1000, poseCallback);
  
  //init time
  ros::Duration(10.0).sleep();

  ROS_INFO("Mooooooved");
  move(10);
  
  
  //Do normal stuff
  ros::spin();
}

void turn(double angle)
{
  //ccw is positive
  double angular_speed = 1;
  double PI = 3.141592653589693;

  //The rotation angle is off the real angle: cf scales the rotation to correct
  double cf = .02/105*abs(angle)+.985-.02/105*30+.002;//linear apprx of correction
  double radians = angle * PI/180*cf;

  geometry_msgs::Pose2D msg;
  geometry_msgs::Pose2D stop;//stop the rotation
  if (angle >= 0)
    msg.theta = angular_speed;
  else
    msg.theta = -angular_speed;
  
  double current_angle = 0;
  double start = ros::Time::now().toSec();
  while (abs(current_angle) < abs(radians))
    {
      current_angle = angular_speed * (ros::Time::now().toSec() - start);
      pub.publish(msg);
      ros::spinOnce();//gives up control for a little bit so laser can scan
    }
  pub.publish(stop);
}

void move(double distance)
{
  double speed = 1;
  geometry_msgs::Pose2D msg;
  geometry_msgs::Pose2D stop;
  msg.x = speed;

  double current_distance = 0;
  double start = ros::Time::now().toSec();
  while (current_distance < distance)
    {
      current_distance = speed * (ros::Time::now().toSec() - start);
      pub.publish(msg);
      ros::spinOnce();//give up control each cycle for distributed system
    }
  pub.publish(stop);
}
