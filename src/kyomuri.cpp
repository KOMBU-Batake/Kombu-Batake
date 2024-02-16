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
  //NcmPoints ncmp = lidar2.getNcmPoints(LiDAR_degree::FRONT, 10);
  //cout << "left count: " << ncmp.count_left << " right count: " << ncmp.count_right << endl;
  //for (auto& p : ncmp.model_left) {
  //  cout << p.x << " " << p.z << endl;
  //}
  //for (auto& p : ncmp.model_right) {
  //  cout << p.x << " " << p.z << endl;
  //}
  RoadAccess access = RecognizingSpaceSimple(lidar2.pointCloud);
  cout << "left count: " << (int)access.front << " left count: " << (int)access.left << " right count: " << (int)access.right << " back count: " << (int)access.back << endl;

  RecognizingSpace(lidar2.pointCloud);
  robot->step(timeStep);
  cout << robot->getTime() << endl;

  cout << "end" << endl;

  delete robot;
  return 0; // ばいばい～
}