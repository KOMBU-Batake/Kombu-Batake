#pragma once
#include "PointCloudLiDAR.h"

struct MAXandMIN {
  float leftMax;
  float leftMin;
  float rightMax;
  float rightMin;
};

// �^�񒆂�left�̖����ɂ���
typedef struct {
  vector<XZcoordinate> model_left;
  vector<XZcoordinate> model_right;
  int count_left;
  int count_right;
} NcmPoints;

// ���̈�Y���p�������
class LiDAR2 :
    public PointCloudLiDAR
{
public:
  // �w�肳�ꂽ���p�𒆐S�Ƃ���N(cm)�̃f�[�^��Ԃ�
  NcmPoints getNcmPoints(const LiDAR_degree& direction, float range);

  WallSet getWallType(const LiDAR_degree& direction);
private:
  // �ő�l�ƍŏ��l���擾����
  MAXandMIN getMAX_MIN(NcmPoints& pointsSet, LiDAR_degree direction);
  void rotateToFront(NcmPoints& pointsSet, LiDAR_degree direction);

  void printLeftRight(const NcmPoints& pointsSet);
  
  XZcoordinate readPoint(int16_t num) {
    if (num < 0) num += 512;
    return pointCloud[num%512];
  }
};
