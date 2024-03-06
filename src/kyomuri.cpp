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

  tank.setDireciton(90, 3);

  for (int i = 0; i < 20; i++) {
    lidar2.update(gps.expectedPos);
    lidar2.getWallType(LiDAR_degree::LEFT);
    tank.gpsTrace(gps.moveTiles(1, 0), 4);
    cout << "-----------------------------" << endl;
  }

  cout << robot->getTime() << endl;

  cout << "end" << endl;

  delete robot;
  return 0; // �΂��΂��`
}