/* 
 * ここはメインな感じのプログラム
 * 実質的なメインプログラムはDFS.cpp
 */

#include "../lib/devices.h"

int main(int argc, char **argv) {

  cout << "start" << endl;
  cout << robot->getTime() << endl;

  try
  {
    // デバイスの有効化
    enableDevices();

    // 深さ優先探索
    //DFS();
    tank.gpsTrace(gps.moveTiles(1, 0), 3);
    tank.setDireciton(180, 3);
    colorsensor.update();
    cout << "left Color: " << (int)colorsensor.getLeftColor() << endl;
    cout << "right Color: " << (int)colorsensor.getRightColor() << endl;
    
    colorCam->saveImage("0.png", 100);
    for (int i = 1; i <= 7; i++) {
      tank.gpsTrace(gps.moveTiles(0, -2), 3);
      cout << "--------" << endl;
      colorsensor.update();
      cout << "left Color: " << (int)colorsensor.getLeftColor() << endl;
      cout << "right Color: " << (int)colorsensor.getRightColor() << endl;
      string filename = to_string(i) + ".png";
      colorCam->saveImage(filename, 100);
    }

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