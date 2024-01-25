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
  // メインルウウウウゥゥゥゥゥゥプ

  cout << "start" << endl;
  cout << myMath.sin.size() << endl;
  cout << myMath.sin[512] << endl;
  cout << myMath.cos.size() << endl;
  cout << myMath.cos[512] << endl;
  mapper.printMap();
  
  tank.gpsTrace(gps.moveTiles(0,7),5);
  tank.gpsTrace(gps.moveTiles(1, 0), 5);
  tank.gpsTrace(gps.moveTiles(0, 1), 5);
  tank.gpsTrace(gps.moveTiles(4, 0), 5);
  tank.gpsTrace(gps.moveTiles(0, 1), 5);
  tank.gpsTrace(gps.moveTiles(2, 0), 5);
  tank.gpsTrace(gps.moveTiles(0, -2), 5);
  tank.gpsTrace(gps.moveTiles(-1, 0), 5);
  tank.gpsTrace(gps.moveTiles(0, -1), 5);
  colorsensor.update();
  ColorHSV hsv = colorsensor.getHSV();
  cout << "H:" << hsv.hue << " S:" << hsv.saturation << " V:" << hsv.value << endl;

  // 深さ優先探索
  //DFS();

  // マップデータ提出

  
  // 終了コマンド
  char message = 'E';
  emitter->send(&message, 1);
  //while (robot->step(timeStep) != -1);

  delete robot;
  return 0; // ばいばい～
}
