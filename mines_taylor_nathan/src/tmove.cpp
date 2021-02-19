/*
  Nathan Taylor
  2/18/21
  Turtle to Trace mines logo
*/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
ros::Publisher pub;

void move(double distance);
void turn(double angle);//ccw

int main(int argc, char **argv)
{
  // INITIALIZE THE NODE
  ros::init(argc, argv, "move_turtle");
  ros::NodeHandle node;
  // Loop at 10Hz, publishing movement commands until we shut down
  ros::Rate rate(10);
  pub = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

  //M Trace Algorithm
  turn(60);//1 not Necessary - alignment for ease of viewing
  move(2);
  turn(-135);//2
  move(2);
  turn(-90);//3
  move(.4);
  turn(90);//4
  move(.8);
  turn(90);//5
  move(1.4);
  turn(90);//6
  move(.8);
  turn(90);//7
  move(.2);
  turn(-90);//8
  move(2);
  turn(-90);//9
  move(.2);
  turn(90);//10
  move(.8);
  turn(90);//11
  move(1.2);
  turn(45);//12
  move(1.6);
  
  turn(-90);//13 HALFWAY ROTATION
  //TAKE IT BACK NOW YALL
  move(1.6);
  turn(45);//12
  move(1.2);
  turn(90);//11
  move(.8);
  turn(90);//10
  move(.2);
  turn(-90);//9
  move(2);
  turn(-90);//9
  move(.2);
  turn(90);//7
  move(.8);
  turn(90);//6
  move(1.4);
  turn(90);//5
  move(.8);
  turn(90);//4
  move(.4);
  turn(-90);//3
  move(2);
  turn(-135);//2
  move(2);
  turn(-45);//1 not Necessary - alignment for ease of viewing
  
  //Do normal stuff
  ros::spin();
}

void turn(double angle)
{
  //ccw is positive
  double angular_speed = 1;
  double PI = 3.141592653589693;

  //The rotation angle must be linearly adjusted to meet the real angle
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
  double speed = 3;
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
