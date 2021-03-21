/*
  Nathan Taylor
  3/20/21
*/
#ifndef _QTABLE_H
#define _QTABLE_H

#include "ros/ros.h"
#include <string>
#include <map>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>

class Q_table
{
public:
  Q_table();
  void readTable(std::ifstream &inFile);
  void writeTable();
  int getReward(std::vector<int> state);
  void updateTable(std::vector<int> p_state,int p_action, std::vector<int> c_state);
  int getAction(std::vector<int> c_state, float e_greedy);
private:
  std::map<std::vector<int>, std::vector<double>> qsa;
};

Q_table::Q_table()
{
  srand(time(NULL)); 
  //build Q-table
  std::ifstream inFile("/home/wit/catkin_ws/src/walls_taylor_nathan/src/qsave.txt");
  if (inFile)//if a saved qtable already exists
    {
      readTable(inFile);
      inFile.close();
    }
  else
    {
      std::vector<double> empty = {0,0,0,0,0};
      for (int i = 0; i < 3; i++)//0
	{
	  for (int j = 0; j < 3; j++)//45
	    {
	      for (int k = 0; k < 3; k++)//90
		{
		  for (int l = 0; l < 3; l++)//135
		    {
		      for (int m = 0; m < 3; m++)//180
			{
			  //0 is close, 1 is medium, 2 is far
			  std::vector<int> temp = {i,j,k,l,m};
			  qsa[temp] = empty;
			}
		    }
		}
	    }
	}
    }
}

void Q_table::readTable(std::ifstream &inFile)
{ 
  for (int i = 0; i < 3; i++)//0
    {
      for (int j = 0; j < 3; j++)//45
	{
	  for (int k = 0; k < 3; k++)//90
	    {
	      for (int l = 0; l < 3; l++)//135
		{
		  for (int m = 0; m < 3; m++)//180
		    {
		      //0 is close, 1 is medium, 2 is far
		      std::vector<int> tstate = {i,j,k,l,m};
		      std::vector<double> values;
		      if (!inFile.eof())//if the file exists, read values in
			{
			  std::string line = "";
			  getline(inFile, line);
			  std::istringstream inStream(line);
			  while (!inStream.eof())
			    {
			      std::string value;
			      inStream >> value;
			      values.push_back(std::stod(value));
			    }
			  qsa[tstate] = values;
			}
		    }
		}
	    }
	}
    }
}

void Q_table::writeTable()
{
  std::ofstream outFile("/home/wit/catkin_ws/src/walls_taylor_nathan/src/qsave.txt");
  for (int i = 0; i < 3; i++)//0
    {
      for (int j = 0; j < 3; j++)//45
	{
	  for (int k = 0; k < 3; k++)//90
	    {
	      for (int l = 0; l < 3; l++)//135
		{
		  for (int m = 0; m < 3; m++)//180
		    {
		      std::vector<int> tstate = {i,j,k,l,m};
		      std::vector<double> tvalues = qsa.at(tstate);
		      std::string svals = std::to_string(tvalues[0]);
		      for (int i = 1; i < tvalues.size(); i++)
			{
			  svals += " " + std::to_string(tvalues[i]);
			}
		      outFile << svals << std::endl;
		    }
		}
	    }
	}
    }
  outFile.close();
}

int Q_table::getReward(std::vector<int> state)
{
  int reward = 0;
  //negative reward for everything except being the right distance away from the left wall
  if (state[0] == 0 || state[1] == 0 || state[2] == 0 ||
      state[3] == 0 || state[4] == 0 || state[1] == 2)
    {
      reward = -1;
    }
  return reward;
}

void Q_table::updateTable(std::vector<int> p_state,int p_action, std::vector<int> c_state)
{
  double a = .2;//learning rate
  double g = .8;//discount factor
  std::vector<double> p_values = qsa.at(p_state);//Q(s,a) 1 out of 243
  std::vector<double> c_values = qsa.at(c_state);
  double p_val = p_values[p_action];//1 of 5
  int reward = getReward(c_state);
  //Find highest potential next value
  int high_index = 0;
  for (int i = 0; i < c_values.size(); i++)
    if (c_values[i] > c_values[high_index])
      	high_index = i;
  double high_next = c_values[high_index];
  
  //set new q-value
  double update = p_val + a * (reward + g * high_next - p_val);
  qsa[p_state][p_action] = update;//not actually old anymore
}

//USE e Greedy
int Q_table::getAction(std::vector<int> c_state, float e_greedy)
{
  double t_index = 0;//index angle to turn
  //int action[5] = {-10,-5,0,5,10};
  std::vector<double> c_values = qsa.at(c_state);
  float r = (float)rand() / RAND_MAX;//Generate random number from 0 to 1
  if (r > e_greedy)
    {
      int high_index = 0;
      for (int i = 0; i < c_values.size(); i++)
	if (c_values[i] > c_values[high_index])
	  high_index = i;
      t_index = high_index;
    }
  else
    {
      int randState = rand() % 5;
      t_index = randState;
    }
  return t_index;
}

#endif
