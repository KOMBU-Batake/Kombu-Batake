#pragma once
#include "PointCloudLiDAR.h"

// �^�񒆂�left�̖����ɂ���
typedef struct {
  vector<XZcoordinate> model_left;
  vector<XZcoordinate> model_right;
  int count_left;
  int count_right;
} NcmPoints;

struct MAXandMIN {
  float leftMax;
  float leftMin;
  float rightMax;
  float rightMin;
};

// ���̈�Y���p�������
class LiDAR2 :
    public PointCloudLiDAR
{
public:
  // �w�肳�ꂽ���p�𒆐S�Ƃ���N(cm)�̃f�[�^��Ԃ�
  NcmPoints getNcmPoints(const LiDAR_degree& direction, uint16_t range);

  WallSet getWallType(const LiDAR_degree& direction);
private:
  // �ő�l�ƍŏ��l���擾����
  MAXandMIN getMAX_MIN(NcmPoints& pointsSet, LiDAR_degree direction);
};
