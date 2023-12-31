// File:          kyomuri.cpp
// Date:
// Description:   For WRS
// Author:        koki0517
// Modifications:

#include <iostream>
#include "devices.h"

using namespace webots;
using namespace std;

// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  enableDevices();
  // ƒƒCƒ“ƒ‹ƒEƒEƒEƒEƒDƒDƒDƒDƒDƒDƒv
  while (robot->step(timeStep) != -1) {
    // double leftDistance = leftToF->getValue();
    // double rightDistance = rightToF->getValue();
    // cout << "Left Distance: " << leftDistance << ", Right Distance: " << rightDistance << endl;
    colorsensor.update();
    cout << colorsensor.getColor() << endl;
    // ColorHSV hsv = colorsensor.getHSV();
    // cout << "Hue: " << hsv.hue << ", Saturation: " << hsv.saturation << ", Value: " << hsv.value << ", RED: " << colorsensor.RGB.red << ", GREEN: " << colorsensor.RGB.green << ", BLUE: " << colorsensor.RGB.blue << endl;
  };

  delete robot;
  return 0; // ‚Î‚¢‚Î‚¢`
}
