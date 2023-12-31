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
  // ���C�����E�E�E�E�D�D�D�D�D�D�v
  while (robot->step(timeStep) != -1) {
    // GPSPosition gpsPosition = gps.getPosition();
    // cout << "x: " << gpsPosition.x << ", y: " << gpsPosition.y << ", z: " << gpsPosition.z << endl;
    cout << leftToF.getDistanceCM() << " , " << rightToF.getDistanceCM() << endl;
  };

  delete robot;
  return 0; // �΂��΂��`
}
