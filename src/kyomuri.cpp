/* 
 * ここはメインな感じのプログラム
 * 実質的なメインプログラムはDFS.cpp
 */

#include <iostream>
#include "../lib/devices.h"

int main(int argc, char **argv) {
  enableDevices();

  cout << "start" << endl;
  cout << robot->getTime() << endl;

  try
  {
    // 深さ優先探索
    //DFS();
    lidar2.update(gps.expectedPos);
    lidar2.getWallType(LiDAR_degree::FRONT);
    lidar2.getWallType(LiDAR_degree::RIGHT);
    lidar2.getWallType(LiDAR_degree::LEFT);
    lidar2.getWallType(LiDAR_degree::BACK);
    cornerSet corner = lidar2.identifyCorner();
    if (corner.front_left) cout << "front_left" << endl;
    if (corner.front_right) cout << "front_right" << endl;
    if (corner.back_left) cout << "back_left" << endl;
    if (corner.back_right) cout << "back_right" << endl;
  }
  catch (...)
  {
    cout << "catch exception" << endl;
    //mapper.replaceLineTo0();
    //mapper.printMap();
    //sendMap(mapper.map_A);
  }

  cout << robot->getTime() << endl;
  cout << "end" << endl;

  delete robot;
  return 0; // ばいばい～
}