#pragma once
#include "PointCloudLiDAR.h"

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
  NcmPoints getNcmPoints(const LiDAR_degree& direction, uint16_t range);

  LiDAR2() = default;

};
