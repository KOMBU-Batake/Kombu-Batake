/* 
 * �����̓��C���Ȋ����̃v���O����
 * �����I�ȃ��C���v���O������DFS.cpp
 */

#include "../lib/devices.h"

int main(int argc, char **argv) {
  try
  {
    // �f�o�C�X�̗L����
    double startTime = robot->getTime();
    cout << "start: " << startTime << endl;
    enableDevices();

    // �[���D��T��
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

  return 0; // �΂��΂��`
}