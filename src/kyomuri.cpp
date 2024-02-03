// File:          kyomuri.cpp
// Date:
// Description:   For WRS
// Author:        koki0517
// Modifications:

#include <iostream>
#include <thread>
#include <mutex>
#include "../lib/devices.h"

using namespace webots;
using namespace std;

// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  enableDevices();
  // ÉÅÉCÉìÉãÉEÉEÉEÉEÉDÉDÉDÉDÉDÉDÉv

  cout << "start" << endl;
  
  //pcLiDAR.update(gps.expectedPos);
  //WallSet wallSet = pcLiDAR.identifyWall(LiDAR_degree::LEFT);
  //cout << (int)wallSet.left << "," << (int)wallSet.center << ", " << (int)wallSet.right << endl;

  // ê[Ç≥óDêÊíTçı
  DFS();

  // É}ÉbÉvÉfÅ[É^íÒèo
  cout << "end" << endl;

  delete robot;
  return 0; // ÇŒÇ¢ÇŒÇ¢Å`
}
