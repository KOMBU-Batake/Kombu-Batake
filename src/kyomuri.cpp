/* 
 * �����̓��C���Ȋ����̃v���O����
 * �����I�ȃ��C���v���O������DFS.cpp
 */

#include "../lib/devices.h"

int main(int argc, char **argv) {

  cout << "start" << endl;
  cout << robot->getTime() << endl;

  try
  {
    // �f�o�C�X�̗L����
    enableDevices();

    // �[���D��T��
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
  return 0; // �΂��΂��`
}