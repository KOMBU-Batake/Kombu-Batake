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
    DFS();

    cout << "end: " << robot->getTime() << endl;
    delete robot;
  }
  catch (...)
  {
    cout << "catch exception" << endl;
    tank.stop(StopMode::HOLD);
    mapper.replaceLineTo0();
    mapper.printMap();
    sendMap(mapper.map_A);
  }

  return 0; // ばいばい～
}