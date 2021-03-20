/*
  Nathan Taylor
  3/20/21
*/
#include "ros/ros.h"
#include <string>
#include <map>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <vector>

class Q_table
{
public:
  Q_table();
  int getReward(std::vector<int> state);
  void updateTable(std::vector<int> p_state,int p_action, std::vector<int> c_state);
  double getAction(std::vector<int> c_state);
private:
  std::map<std::vector<int>, std::vector<double>> qsa;
  float e;//greeeeddyyyyyy
};

Q_table::Q_table()
{
  //build Q-table
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
    srand(time(NULL));
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

//DEFINE STATE ACTION PAIRS - SWITCH TO QLEARNING
double Q_table::getAction(std::vector<int> c_state)
{
  double turn = 0;//angle to turn
  int action[5] = {-10,-5,0,5,10};
  std::vector<double> c_values = qsa.at(c_state);
  float r = (float)rand() / RAND_MAX;//Generate random number from 0 to 1
  if (r > e)
    {
      int high_index = 0;
      for (int i = 0; i < c_values.size(); i++)
	if (c_values[i] > c_values[high_index])
	  high_index = i;
      turn = action[high_index];
    }
  else
    {
      int randState = rand() % 5;
      turn = action[randState];
    }
  return turn;
}
