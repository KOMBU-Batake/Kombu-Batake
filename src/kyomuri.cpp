#include <iostream>
#include "../lib/devices.h"

int main(int argc, char **argv) {
  enableDevices();

  cout << "start" << endl;

  // [‚³—Dæ’Tõ
  //DFS();
  tank.setDireciton(185, 3);
  lidar2.update(gps.expectedPos);
  NcmPoints ncmp = lidar2.getNcmPoints(LiDAR_degree::FRONT, 10);
  cout << "left count: " << ncmp.count_left << " right count: " << ncmp.count_right << endl;
  for (auto& p : ncmp.model_left) {
    cout << p.x << " " << p.z << endl;
  }
  for (auto& p : ncmp.model_right) {
    cout << p.x << " " << p.z << endl;
  }

  cout << "end" << endl;

  delete robot;
  return 0; // ‚Î‚¢‚Î‚¢`
}