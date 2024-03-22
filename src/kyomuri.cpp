/* 
 * �����̓��C���Ȋ����̃v���O����
 * �����I�ȃ��C���v���O������DFS.cpp
 */

#include "../lib/devices.h"

int main(int argc, char **argv) {
  try
  {
    // �f�o�C�X�̗L����
    enableDevices();

    cout << "start: " << robot->getTime() << endl;

    // �[���D��T��
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

  return 0; // �΂��΂��`
}