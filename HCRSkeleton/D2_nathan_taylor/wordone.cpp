#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main()
{
  ifstream inFile("cust_d2.t");
  string line = "";
  while (!inFile.eof())
    {
      getline(inFile, line);
      cout << line[0] << endl;
    }
  inFile.close();
  return 0;
}
