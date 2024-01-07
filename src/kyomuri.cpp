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
  //robot->step(5000);

  cout << "start" << endl;
  //tank.setDireciton(270, 6);
  //tank.setPosition(180, 180, -2, 2);

  while (robot->step(timeStep) != -1) {
    tank.setDireciton(350, 3);
    cout << "1" << endl;
    tank.setDireciton(90, 3);
    cout << "2" << endl;
    tank.setDireciton(180, 3);
    cout << "3" << endl;
    tank.setDireciton(270, 3);
    cout << "4" << endl;
    tank.setDireciton(30, 3);
    cout << "5" << endl;
    tank.setDireciton(270, 3);
    /*GPSPosition pos = { -48,0 };
    tank.gpsTraceSimple(pos, 3, Direction_of_Travel::x);*/
    while (robot->step(timeStep) != -1);
  };

  delete robot;
  return 0; // ÇŒÇ¢ÇŒÇ¢Å`
}
