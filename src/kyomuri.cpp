// File:          kyomuri.cpp
// Date:
// Description:   For WRS
// Author:        koki0517
// Modifications:

#include <iostream>
#include <thread>
#include <mutex>
#include "../lib/devices.h"

using namespace webots;
using namespace std;

// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  enableDevices();
  // ���C�����E�E�E�E�D�D�D�D�D�D�v

  cout << "start" << endl;
  pcDetails model_1 = { LiDAR_degree::FRONT,TagDegree::leftLarge_rightLarge, TagMinDistance::one_point_five, {0,1,2} };
  cout << model_1.model[0] << model_1.model[1] << model_1.model[2] << endl;
  /*pcLiDAR.update();
  for (int i = 0; i < 512; i++) {
    cout << pcLiDAR.pointCloud[i].x << "," << pcLiDAR.pointCloud[i].z << endl;
  }*/

  // �[���D��T��
  //DFS();

  // �}�b�v�f�[�^��o
  robot->step(timeStep*1000);
  
  // �I���R�}���h
  char message = 'E';
  emitter->send(&message, 1);
  //while (robot->step(timeStep) != -1);

  delete robot;
  return 0; // �΂��΂��`
}
