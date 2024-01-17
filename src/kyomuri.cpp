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

  while (robot->step(timeStep) != -1) {
    //GPSPosition pos0 = gps.moveTiles(0, 2);
    //tank.gpsTrace(pos0, 4, Direction_of_Travel::z,StopMode::COAST);
    //cout << "a" << endl;
    //GPSPosition pos = gps.moveTiles(0, 1);
    //tank.gpsTrace(pos, 3, Direction_of_Travel::z);
    //GPSPosition pos1 = gps.moveTiles(2, 0);
    //tank.gpsTrace(pos1, 3, Direction_of_Travel::diagonal);
    //GPSPosition pos2 = gps.moveTiles(-1, -2);
    //tank.gpsTrace(pos2, 3, Direction_of_Travel::diagonal);
    cout << robot->getTime() << endl;
    //while (robot->step(timeStep) != -1);
  };

  delete robot;
  return 0; // ÇŒÇ¢ÇŒÇ¢Å`
}
