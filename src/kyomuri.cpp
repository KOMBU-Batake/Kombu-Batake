#include <iostream>
#include "../lib/devices.h"

int main(int argc, char **argv) {
  enableDevices();

  cout << "start" << endl;

  // [‚³—Dæ’Tõ
  //DFS();
  lidar2.update(gps.expectedPos);



  cout << "end" << endl;

  delete robot;
  return 0; // ‚Î‚¢‚Î‚¢`
}