/* 
 * �����̓��C���Ȋ����̃v���O����
 * �����I�ȃ��C���v���O������DFS.cpp
 */

#include <iostream>
#include "../lib/devices.h"

int main(int argc, char **argv) {
  enableDevices();

  cout << "start" << endl;

  // �[���D��T��
  //DFS();
  cout << robot->getTime() << endl;
  lidar2.update(gps.expectedPos);

  lidar2.getWallType(LiDAR_degree::RIGHT);

  robot->step(timeStep);
  cout << robot->getTime() << endl;

  cout << "end" << endl;

  delete robot;
  return 0; // �΂��΂��`
}