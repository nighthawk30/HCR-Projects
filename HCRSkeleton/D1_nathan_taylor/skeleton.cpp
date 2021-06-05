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

int main()
{
  //labeling system: action, subject, trial number
  //a12_s08_e02-
  //data
  //train: a08-16_s01-06_e01-02_skeleton_proj.txt_
  //frame joint x y z
  vector<string> mode = {"train","test"};
  vector<string> mwrite = {"_d1", "_d1.t"};
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
      string oname1 = "rad" + mwrite[l];
      string oname2 = "cust" + mwrite[l];
      ofstream outFile1(oname1);
      ofstream outFile2(oname2);
      
      for (int j = 0; j < subject[l].size(); j++)
	for (int k = 0; k < trial.size(); k++)
	  for (int i = 0; i < actions.size(); i++)
	    {
	      string fname = "a"+actions[i]+"_s"+subject[l][j]+"_e"+trial[k]+"_skeleton_proj.txt";
	      string fpath = "dataset/" + mode[l] + "/" + fname;

	      //get the instance
	      map<int, map<int, vector<double>>> instance = parseFile(fpath);//holds file data
	      //This allows usage without regard for order of data
	      outFile1 << RAD(instance) << endl;
	      outFile2 << D20(instance) << endl;
	      //Now loop through frames
	      
	    }
      outFile1.close();
      outFile2.close();
}

  //DEALING WITH NaN
  //since were are using a map of map of vectors, we can just delete the map corresponding to the frame where the NaN value was found
  /*
  string fpath = "dataset/train/a13_s06_e02_skeleton_proj.txt";
  map<int, map<int, vector<double>>> instance = parseFile(fpath);
  ofstream outFile("nan.txt");
  outFile << RAD(instance) << endl;
  */  
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
  vector<vector<double>> distang(10);//5 distances, 5 angles, 1 vector

  //loop through frames and get all data into 10 sets
  for (int i = 1; i <= instance.size(); i++)
    {
      //distances for 1 frame
      distang[0].push_back(distance3D(instance[i][1],instance[i][8]));
      distang[1].push_back(distance3D(instance[i][1],instance[i][16]));
      distang[2].push_back(distance3D(instance[i][1],instance[i][20]));
      distang[3].push_back(distance3D(instance[i][1],instance[i][12]));
      distang[4].push_back(distance3D(instance[i][1],instance[i][4]));
      distang[5].push_back(angle3D(instance[i][1],instance[i][8],instance[i][16]));
      distang[6].push_back(angle3D(instance[i][1],instance[i][16],instance[i][20]));
      distang[7].push_back(angle3D(instance[i][1],instance[i][20],instance[i][12]));
      distang[8].push_back(angle3D(instance[i][1],instance[i][12],instance[i][4]));
      distang[9].push_back(angle3D(instance[i][1],instance[i][4],instance[i][8]));
    }
	      
  //sort needed for max and min
  for (int i = 0; i < 10; i++)
      sort(distang[i].begin(), distang[i].end());

  //lets fix the bin count
  int binCount = 15;
  vector<double> histograms;
  for (int i = 0; i < 10; i++)
    {
      vector<double> bins(binCount, 0);
      double range = distang[i][distang[i].size() - 1] - distang[i][0];
      double width = range/binCount;
  
      for (int j = 0; j < distang[i].size(); j++)
	{
	  //max bin range is low value + bin number * bin width
	  //find proper bin for data point
	  for (int k = 0; k < binCount; k++)
	    {
	      double tvalue = distang[i][j] - distang[i][0];
	      if (tvalue >= width * k && tvalue <= width * (k + 1))
		{
		  bins[k] += 1/(double)instance.size();//add normalized value to bin
		}
	    }
	}
      histograms.insert(histograms.end(), bins.begin(), bins.end());
    }

  string radbars = to_string(histograms[0]);
  for (int i = 1; i < histograms.size(); i++)
      radbars += " " + to_string(histograms[i]);
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
  for (int i = 0; i < 10; i++)
      sort(distance[i].begin(), distance[i].end());

  int binCount = sqrt(instance.size()) + 1;
  vector<double> histograms;
  for (int i = 0; i < 10; i++)
    {
      vector<double> bins(binCount, 0);
      double range = distance[i][distance[i].size() - 1] - distance[i][0];
      double width = range/binCount;
  
      for (int j = 0; j < distance[i].size(); j++)
	{
	  //max bin range is low value + bin number * bin width
	  //find proper bin for data point
	  for (int k = 0; k < binCount; k++)
	    {
	      double tvalue = distance[i][j] - distance[i][0];
	      if (tvalue >= width * k && tvalue <= width * (k + 1))
		{
		  bins[k] += 1/(double)instance.size();//add normalized value to bin
		}
	    }
	}
      histograms.insert(histograms.end(), bins.begin(), bins.end());
    }

  string distbars = to_string(histograms[0]);
  for (int i = 1; i < histograms.size(); i++)
      distbars += " " + to_string(histograms[i]);
  return distbars;
}
