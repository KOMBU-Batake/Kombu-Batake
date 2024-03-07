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

  try
  {
    tank.setDireciton(90, 3);
    for (int i = 0; i < 20; i++) {
      lidar2.update(gps.expectedPos);
      lidar2.getWallType(LiDAR_degree::LEFT);
      tank.gpsTrace(gps.moveTiles(1, 0), 4);
      cout << "-----------------------" << endl;
    }
    lidar2.update(gps.expectedPos);
    lidar2.getWallType(LiDAR_degree::FRONT);
  }
  catch (...)
  {
    cout << "catch exception" << endl;
  }


  cout << robot->getTime() << endl;

  cout << "end" << endl;

  delete robot;
  return 0; // ばいばい〜
}