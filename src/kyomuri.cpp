#include <iostream>
#include "../lib/devices.h"

int main(int argc, char **argv) {
  enableDevices();

  cout << "start" << endl;

  // �[���D��T��
  //DFS();
  lidar2.update(gps.expectedPos);



  cout << "end" << endl;

  delete robot;
  return 0; // �΂��΂��`
}