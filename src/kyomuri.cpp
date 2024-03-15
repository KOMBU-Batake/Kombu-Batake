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
    DFS();
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