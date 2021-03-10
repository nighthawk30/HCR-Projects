/*
  Nathan Taylor
*/
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
//#include "geometry_msgs/Twist.h"

#include <cmath>
ros::Publisher pub;
/*
void move(double distance);
void turn(double angle);//ccw
*/
int main(int argc, char **argv)
{
  // INITIALIZE THE NODE
  ros::init(argc, argv, "move_turtle");
  ros::NodeHandle node;
  // Loop at 10Hz, publishing movement commands until we shut down
  ros::Rate rate(10);
  pub = node.advertise<geometry_msgs::Pose2D>("/triton_lidar/vel_cmd", 10);

  //init time
  ros::Duration(10.0).sleep();


  geometry_msgs::Pose2D msg;
  msg.x = 1;
  pub.publish(msg);  

  
  //Do normal stuff
  ros::spin();
}
/*
void turn(double angle)
{
  //ccw is positive
  double angular_speed = 1;
  double PI = 3.141592653589693;

  //The rotation angle is off the real angle: cf scales the rotation to correct
  double cf = .02/105*abs(angle)+.985-.02/105*30+.002;//linear apprx of correction
  double radians = angle * PI/180*cf;

  geometry_msgs::Twist msg;
  geometry_msgs::Twist stop;//stop the rotation
  if (angle >= 0)
    msg.angular.z = angular_speed;
  else
    msg.angular.z = -angular_speed;
  
  double current_angle = 0;
  double start = ros::Time::now().toSec();
  while (abs(current_angle) < abs(radians))
    {
      current_angle = angular_speed * (ros::Time::now().toSec() - start);
      pub.publish(msg);
    }
  pub.publish(stop);
}

void move(double distance)
{
  double speed = 4;
  geometry_msgs::Twist msg;
  geometry_msgs::Twist stop;
  msg.linear.x = speed;

  double current_distance = 0;
  double start = ros::Time::now().toSec();
  while (current_distance < distance)
    {
      current_distance = speed * (ros::Time::now().toSec() - start);
      pub.publish(msg);
    }
  pub.publish(stop);
}
*/
