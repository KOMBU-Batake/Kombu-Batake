/* 
 * ここはメインな感じのプログラム
 * 実質的なメインプログラムはDFS.cpp
 */

#include <iostream>
#include "../lib/devices.h"

int main(int argc, char **argv) {
  enableDevices();

  cout << "start" << endl;

  // 深さ優先探索
  //DFS();
  cout << robot->getTime() << endl;
  lidar2.update(gps.expectedPos);

  lidar2.getWallType(LiDAR_degree::RIGHT);

  robot->step(timeStep);
  cout << robot->getTime() << endl;

  cout << "end" << endl;

  delete robot;
  return 0; // ばいばい～
}