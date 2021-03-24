/*
  Nathan Taylor
  3/21/21
*/

#ifndef _SETSTATE_H
#define _SETSTATE_H

#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
#include "std_msgs/String.h"
#include <string>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <ctime>

class SetState
{
public:
  SetState();
  gazebo_msgs::SetModelState resetState();
  std::vector<double> toQuaternion(std::vector<double> ypr);
private:
  double PI = 3.141592653589693;
  std::vector<std::pair<double,double>> start_poses;
};

SetState::SetState()
{
  srand(time(NULL));
  //list of starting locations for training
  start_poses.push_back(std::pair<double,double>(3.7,3.7));
  start_poses.push_back(std::pair<double,double>(3,3.5));
  start_poses.push_back(std::pair<double,double>(3.7,0));
  start_poses.push_back(std::pair<double,double>(2.3,-2));
  start_poses.push_back(std::pair<double,double>(-3,1.5));
  start_poses.push_back(std::pair<double,double>(-3.5,1));
  start_poses.push_back(std::pair<double,double>(-1.5,.3));
  start_poses.push_back(std::pair<double,double>(0,-2.3));
  start_poses.push_back(std::pair<double,double>(0,-1.5));
  start_poses.push_back(std::pair<double,double>(-3.7,-.3));
  start_poses.push_back(std::pair<double,double>(-3.7,0.3));
  start_poses.push_back(std::pair<double,double>(2.3,1.5));
}

gazebo_msgs::SetModelState SetState::resetState()
{
  int rstate = rand() % start_poses.size();
  gazebo_msgs::SetModelState reset;
  reset.request.model_state.model_name = "triton_lidar";
  reset.request.model_state.pose.position.x = start_poses[rstate].first;
  reset.request.model_state.pose.position.y = start_poses[rstate].second;
  std::vector<double> q_msg = toQuaternion({(double)(rand() % 360),0,0});//choose random orientation
  reset.request.model_state.pose.orientation.w = q_msg[0];
  reset.request.model_state.pose.orientation.x = q_msg[1];
  reset.request.model_state.pose.orientation.y = q_msg[2];
  reset.request.model_state.pose.orientation.z = q_msg[3];
  return reset;
}

std::vector<double> SetState::toQuaternion(std::vector<double> ypr)
{
  std::vector<double> quat;
  //from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  double cy = cos(ypr[0] * 0.5 * PI/180);
  double sy = sin(ypr[0] * 0.5 * PI/180);
  double cp = cos(ypr[1] * 0.5 * PI/180);
  double sp = sin(ypr[1] * 0.5 * PI/180);
  double cr = cos(ypr[2] * 0.5 * PI/180);
  double sr = sin(ypr[2] * 0.5 * PI/180);
  quat.push_back(cr * cp * cy + sr * sp * sy);//w
  quat.push_back(sr * cp * cy - cr * sp * sy);//x
  quat.push_back(cr * sp * cy + sr * cp * sy);//y
  quat.push_back(cr * cp * sy - sr * sp * cy);//z
  return quat;
}

#endif
