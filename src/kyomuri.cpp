/* 
 * ここはメインな感じのプログラム
 * 実質的なメインプログラムはDFS.cpp
 */

#include "../lib/devices.h"

int main(int argc, char **argv) {
  try
  {
    // デバイスの有効化
    enableDevices();

    cout << "start: " << robot->getTime() << endl;

    // 深さ優先探索
    //DFS();

    //tank.setDireciton(180, 3);
    //colorCam->saveImage("1.png", 100);
    //lidar2.update(gps.expectedPos);
    //colorsensor.update();
    //colorsensor.obstacle();
    myCam.update();
    vector<bool> result = myCam.leftHole();
    cout << "leftHole: " << result[0] << ", " << result[1] << endl;
    vector<bool> result2 = myCam.rightHole();
    cout << "rightHole: " << result2[0] << ", " << result2[1] << endl;
    tank.gpsTrace(gps.moveTiles(0, -1), 3);
    result = myCam.leftHole();
    cout << "leftHole: " << result[0] << ", " << result[1] << endl;
    result2 = myCam.rightHole();
    cout << "rightHole: " << result2[0] << ", " << result2[1] << endl;
    tank.gpsTrace(gps.moveTiles(0, -1), 3);
    tank.gpsTrace(gps.moveTiles(-2, 0), 3);
    myCam.update();
    result = myCam.leftHole();
    cout << "leftHole: " << result[0] << ", " << result[1] << endl;
    leftCam->saveImage("left.png", 100);

    cout << "end: " << robot->getTime() << endl;
    delete robot;
  }
  catch (...)
  {
    cout << "catch exception" << endl;
    //mapper.replaceLineTo0();
    //mapper.printMap();
    //sendMap(mapper.map_A);
  }

  return 0; // ばいばい～
}