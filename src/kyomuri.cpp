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
    //DFS();

    tank.setDireciton(180, 3);
    colorCam->saveImage("1.png", 100);
    lidar2.update(gps.expectedPos);
    colorsensor.update();
    colorsensor.obstacle();
    cout << "-------------------" << endl;
    for (auto& p : lidar2.pointCloud) {
			cout << p.x << ", " << p.z << endl;
		}

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

  return 0; // �΂��΂��`
}