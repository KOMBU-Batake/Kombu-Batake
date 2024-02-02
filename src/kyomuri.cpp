// File:          kyomuri.cpp
// Date:
// Description:   For WRS
// Author:        koki0517
// Modifications:

#include <iostream>
#include <thread>
#include <mutex>
#include "../lib/devices.h"
#include <opencv2/opencv.hpp>

using namespace webots;
using namespace std;

// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  enableDevices();
  // メインルウウウウゥゥゥゥゥゥプ

  cout << "start" << endl;
  
  //pcLiDAR.update(gps.expectedPos);
  //WallSet wallSet = pcLiDAR.identifyWall(LiDAR_degree::LEFT);
  //cout << (int)wallSet.left << "," << (int)wallSet.center << ", " << (int)wallSet.right << endl;

  // 深さ優先探索
  DFS();

  // マップデータ提出
  cout << "end" << endl;
  
  // 終了コマンド
  char message = 'E';
  emitter->send(&message, 1);
  //while (robot->step(timeStep) != -1);

  delete robot;
  return 0; // ばいばい～
}
