#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <cmath>
#include <map>
#include <set>

using namespace std;

double distance3D(vector<double> p1, vector<double> p2);
double angle3D(vector<double> p0, vector<double> p1, vector<double> p2);
string RAD(map<int, map<int, vector<double>>> instance);
map<int, map<int, vector<double>>> parseFile(string fpath);
string D20(map<int, map<int, vector<double>>> instance);
string RAD2(map<int, map<int, vector<double>>> instance);


int main()
{
  //labeling system: action, subject, trial number
  //a12_s08_e02-
  //data
  //train: a08-16_s01-06_e01-02_skeleton_proj.txt_
  //frame joint x y z
  vector<string> mode = {"train","test"};
  vector<string> mwrite = {"_d2", "_d2.t"};
  //actions indices          0     1    2   3    4    5
  vector<string> actions = {"08","10","12","13","15","16"};
  vector<string> sub1 = {"01","02","03","04","05","06"};
  vector<string> sub2 = {"07","08","09","10"};
  vector<vector<string>> subject;//subject set depends on test or train
  subject.push_back(sub1);
  subject.push_back(sub2);
  vector<string> trial = {"01","02"};
  
  //build file string
  for (int l = 0; l < mode.size(); l++)
    {
      string oname1 = "../rad" + mwrite[l];
      string oname2 = "../cust" + mwrite[l];
      ofstream outFile1(oname1);
      ofstream outFile2(oname2);
      //ofstream outFile("d_tune.txt");
      
      for (int j = 0; j < subject[l].size(); j++)
	for (int k = 0; k < trial.size(); k++)
	  for (int i = 0; i < actions.size(); i++)
	    {
	      string fname = "a"+actions[i]+"_s"+subject[l][j]+"_e"+trial[k]+"_skeleton_proj.txt";
	      string fpath = "../dataset/" + mode[l] + "/" + fname;

	      //get the instance
	      map<int, map<int, vector<double>>> instance = parseFile(fpath);//holds file data
	      //This allows usage without regard for order of data

	      //adds action onto the front for svm
	      outFile1 << to_string(i)+RAD(instance) << endl;
	      outFile2 << to_string(i)+D20(instance) << endl;
	    }
      outFile1.close();
      outFile2.close();
    }
  return 0;
}

map<int, map<int, vector<double>>> parseFile(string fpath)
{
  set<int> NaNFrames;
  //get the instance
  map<int, map<int, vector<double>>> instance;//holds file data
  //This allows usage without regard for order of data
	  
  ifstream inFile(fpath);//relative paths fuck yeah
  if (inFile)
    {
      //Parse the file data
      while (!inFile.eof())
	{
	  string line;
	  getline(inFile, line);
	  if (line != "")//a precaution
	    {
	      istringstream inStream(line);
		  
	      //classification data
	      string sframe;
	      string sjoint;
	      inStream >> sframe;
	      inStream >> sjoint;
	      int frame = stoi(sframe);
	      int joint = stoi(sjoint);

	      for (int i = 0; i < 3; i++)//get point data
		{
		  string partial;
		  inStream >> partial;
		  if (partial == "NaN")
		    NaNFrames.insert(frame);
		  instance[frame][joint].push_back(stod(partial));//powerful 2d indexing
		}
	    }
	}
      
      for (set<int>::iterator it = NaNFrames.begin(); it != NaNFrames.end(); it++)  
	instance.erase(*it);//get rid of frames with NaN values
      inFile.close();
    }
  return instance;
}

double distance3D(vector<double> p1, vector<double> p2)
{
  double sum = 0;
  for (int i = 0; i < 3; i++)
    {
      sum += pow(p1[i]-p2[i],2);
    }
  return sqrt(sum);
}

double angle3D(vector<double> p0, vector<double> p1, vector<double> p2)
{
  double d1 = distance3D(p0, p1);
  double d2 = distance3D(p0, p2);
  double dot = 0;
  for (int i = 0; i < 3; i++)
    {
      dot += (p1[i]-p0[i]) * (p2[i]-p0[i]);
    }
  return acos(dot/(d1*d2)) * 180 / (3.141592653587932);
}

//computes relative angles and distances for all frames in a file then creates histograms
string RAD(map<int, map<int, vector<double>>> instance)
{
  //relevant joints: C:1 8 16 20 12 4
  //cant use sets because we need duplicates
  vector<vector<double>> distance(5);//5 distances, 5 angles, 1 vector
  vector<vector<double>> angle(5);
  
  //loop through frames and get all data into 10 sets
  for (int i = 1; i <= instance.size(); i++)
    {
      //distances for 1 frame
      distance[0].push_back(distance3D(instance[i][1],instance[i][8]));
      distance[1].push_back(distance3D(instance[i][1],instance[i][16]));
      distance[2].push_back(distance3D(instance[i][1],instance[i][20]));
      distance[3].push_back(distance3D(instance[i][1],instance[i][12]));
      distance[4].push_back(distance3D(instance[i][1],instance[i][4]));
      angle[0].push_back(angle3D(instance[i][1],instance[i][8],instance[i][16]));
      angle[1].push_back(angle3D(instance[i][1],instance[i][16],instance[i][20]));
      angle[2].push_back(angle3D(instance[i][1],instance[i][20],instance[i][12]));
      angle[3].push_back(angle3D(instance[i][1],instance[i][12],instance[i][4]));
      angle[4].push_back(angle3D(instance[i][1],instance[i][4],instance[i][8]));
    }
	      
  //sort needed for max and min - MAYBE NOT NECESSARY
  /*
  for (int i = 0; i < 5; i++)
    {
      sort(distance[i].begin(), distance[i].end());
      sort(angle[i].begin(), angle[i].end());
    }
  */
  //Hyper parameters
  //Max angle: 180
  //Min angle: 0
  //Max dist: 2 but 1 is most common
  //Min dist: 0

  //0 - max + 1 overflow bin
  double dmax = 1.1;
  int dbins = 15;
  double dwidth = dmax/(double)dbins;
  double amax = 180;
  int abins = 15;
  double awidth = amax/(double)abins;

  vector<double> histograms;

  for (int i = 0; i < 5; i++)//loop through ranges
    {
      vector<double> bins(abins, 0);
      for (int j = 0; j < angle[i].size(); j++)//loop through pts in data for 1 range
	{
	  for (int k = abins - 1; k >= 0; k--)//which bin does each pt fall into
	    {
	      if (angle[i][j] >= awidth * k)
		{
		  bins[k] += 1/(double)instance.size();
		  break;
		}
	    }
	}
      histograms.insert(histograms.end(), bins.begin(), bins.end());      
    }

  //distance histograms
  for (int i = 0; i < 5; i++)
    {
      vector<double> bins(dbins, 0);
  
      for (int j = 0; j < distance[i].size(); j++)
	{
	  //max bin range is low value + bin number * bin width
	  //find proper bin for data point
	  for (int k = dbins - 1; k >= 0; k--)//count down
	    {
	      if (distance[i][j] >= dwidth * k)//last bin becomes overflow
		{
		  bins[k] += 1/(double)instance.size();//add normalized value to bin
		  break;//dont add on to higher bins as well
		}
	    }
	}
      histograms.insert(histograms.end(), bins.begin(), bins.end());
    }

  //convert to string
  string radbars;
  for (int i = 0; i < histograms.size(); i++)
    radbars += " " + to_string(i+1) + ":" + to_string(histograms[i]);
  return radbars;
}

//computes all 20 distances for all frames in a file then creates histograms
string D20(map<int, map<int, vector<double>>> instance)
{
  //relevant joints: C:1 8 16 20 12 4
  //cant use sets because we need duplicates
  vector<vector<double>> distance(19);//19distances from center

  //loop through frames and get all data into 10 sets
  for (int i = 1; i <= instance.size(); i++)//all instances
    {
      for (int j = 2; j <= 20; j++)
	{
	  distance[j-2].push_back(distance3D(instance[i][1],instance[i][j]));
	}
    }
	      
  //sort needed for max and min
  /*
  for (int i = 0; i < 10; i++)
      sort(distance[i].begin(), distance[i].end());
  */
  
  double dmax = 1.1;
  int dbins = 15;
  double dwidth = dmax/(double)dbins;

  vector<double> histograms;

  //distance histograms
  for (int i = 0; i < 19; i++)
    {
      vector<double> bins(dbins, 0);
  
      for (int j = 0; j < distance[i].size(); j++)
	{
	  //max bin range is low value + bin number * bin width
	  //find proper bin for data point
	  for (int k = dbins - 1; k >= 0; k--)//count down
	    {
	      if (distance[i][j] >= dwidth * k)//last bin becomes overflow
		{
		  bins[k] += 1/(double)instance.size();//add normalized value to bin
		  break;//dont add on to higher bins as well
		}
	    }
	}
      histograms.insert(histograms.end(), bins.begin(), bins.end());
    }

  string distbars;
  for (int i = 0; i < histograms.size(); i++)
    distbars += " " + to_string(i+1) + ":" + to_string(histograms[i]);
  return distbars;
}
