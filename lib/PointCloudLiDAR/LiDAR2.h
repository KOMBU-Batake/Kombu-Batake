#pragma once
#include "PointCloudLiDAR.h"

// �^�񒆂�left�̖����ɂ���
struct NcmPoints {
  vector<float> model_left;
  vector<float> model_right;
	int count_left;
	int count_right;
};

// ���̈�Y���p�������
class LiDAR2 :
    public PointCloudLiDAR
{
public:
  // �w�肳�ꂽ���p�𒆐S�Ƃ���N(cm)�̃f�[�^��Ԃ�
  NcmPoints getNcmPoints(const LiDAR_degree& direction, uint16_t range);

  LiDAR2() = default;
};

