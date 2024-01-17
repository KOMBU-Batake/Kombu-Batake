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

void task1() {
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
    //while (robot->step(timeStep) != -1);
    //this_thread::sleep_for(100ms);
  };
}

void task2() {
  while (robot->step(timeStep) != -1) {
    GPSPosition pos0 = gps.moveTiles(0, 2);
    tank.gpsTrace(pos0, 4);
    GPSPosition pos = gps.moveTiles(1, 3);
    tank.gpsTrace(pos, 3);
    GPSPosition pos1 = gps.moveTiles(2, 0);
    tank.gpsTrace(pos1, 3);
    GPSPosition pos2 = gps.moveTiles(-1, -2);
    tank.gpsTrace(pos2, 3);
    while (robot->step(timeStep) != -1);
  };
}

// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // ÉÅÉCÉìÉãÉEÉEÉEÉEÉDÉDÉDÉDÉDÉDÉv
  cout << "start" << endl;
  enableDevices();

  while (robot->step(timeStep) != -1) {
    gps.recoedStartPosition();

    thread th1(task2);
    thread th2(task1);
    th1.join();
    th2.join();
    cout << "end" << endl;
    while (robot->step(timeStep) != -1);
  };

  delete robot;
  return 0; // ÇŒÇ¢ÇŒÇ¢Å`
}
