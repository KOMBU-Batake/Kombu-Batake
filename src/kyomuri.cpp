/* 
 * ここはメインな感じのプログラム
 * 実質的なメインプログラムはDFS.cpp
 */

#include "../lib/devices.h"

int main(int argc, char **argv) {
  try
  {
    // デバイスの有効化
    double startTime = robot->getTime();
    cout << "start: " << startTime << endl;
    enableDevices();

    // 深さ優先探索
    DFS(startTime);

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