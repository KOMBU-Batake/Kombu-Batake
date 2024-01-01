// File:          kyomuri.cpp
// Date:
// Description:   For WRS
// Author:        koki0517
// Modifications:

#include <iostream>
#include "lib/devices.h"

using namespace webots;
using namespace std;

// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  enableDevices();
  // ÉÅÉCÉìÉãÉEÉEÉEÉEÉDÉDÉDÉDÉDÉDÉv
  //robot->step(5000);
  cout << "start" << endl;
  tank.setPosition(360, 360, 2, 2, false);
  tank.setPosition(180, -180, 2, 2, false);
  cout << "fin" << endl;
  cout << "left position: " << tank.getLeftEncoder() << ", right position: " << tank.getRightEncoder() << endl;
  while (robot->step(timeStep) != -1) {
    
  };

  delete robot;
  return 0; // ÇŒÇ¢ÇŒÇ¢Å`
}
