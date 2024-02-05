// File:          kyomuri.cpp
// Date:
// Description:   For WRS
// Author:        koki0517
// Modifications:

#include <iostream>
#include "../lib/devices.h"

using namespace webots;
using namespace std;

int main(int argc, char **argv) {
  enableDevices();

  cout << "start" << endl;

  // [‚³—Dæ’Tõ
  //DFS();
  
  pcLiDAR.update(gps.expectedPos);
  cout << robot->getTime() << endl;
  RoadAccess access = RecognizingSpace(pcLiDAR.pointCloud);
  cout << "front: " << (int)access.front << ", left: " << (int)access.left << ", right: " << (int)access.right << ", back: " << (int)access.back << endl;
  cout << robot->getTime() << endl;

  cout << "end" << endl;

  delete robot;
  return 0; // ‚Î‚¢‚Î‚¢`
}
