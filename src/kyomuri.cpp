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
  // ƒƒCƒ“ƒ‹ƒEƒEƒEƒEƒDƒDƒDƒDƒDƒDƒv
  //robot->step(5000);

  cout << "start" << endl;
  tank.setDireciton(270, 6);
  //tank.setPosition(180, 180, -2, 2);
  while (robot->step(timeStep) != -1) {
    //cout << "gyro; " << gyro.getGyro() << endl;
  };

  delete robot;
  return 0; // ‚Î‚¢‚Î‚¢`
}
