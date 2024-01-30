// File:          kyomuri.cpp
// Date:
// Description:   For WRS
// Author:        koki0517
// Modifications:

#include <iostream>
#include <thread>
#include <mutex>
#include "../lib/devices.h"
//#include <opencv2/opencv.hpp>

using namespace webots;
using namespace std;

// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  enableDevices();
  // メインルウウウウゥゥゥゥゥゥプ

  cout << "start" << endl;
  //pcLiDAR.update();
  //for (int i = 0; i < 512; i++) {
  //  cout << pcLiDAR.pointCloud[i].x << "," << pcLiDAR.pointCloud[i].z << endl;
  //}
  pcLiDAR.modelSamplimg();
  //tank.setDireciton(90,5);
  //pcLiDAR.update(gps.expectedPos);
  //pcLiDAR.identifyWall(LiDAR_degree::LEFT);
  //pcLiDAR.identifyWall(LiDAR_degree::RIGHT);
  //pcLiDAR.identifyWall(LiDAR_degree::LEFT);
  //pcLiDAR.identifyWall(LiDAR_degree::BACK);

  // 深さ優先探索
  //DFS();

  // マップデータ提出
  cout << "end" << endl;
  
  // 終了コマンド
  char message = 'E';
  emitter->send(&message, 1);
  //while (robot->step(timeStep) != -1);

  delete robot;
  return 0; // ばいばい～
}
