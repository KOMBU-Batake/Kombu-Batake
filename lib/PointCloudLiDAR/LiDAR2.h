#pragma once
#include "PointCloudLiDAR.h"

// 真ん中はleftの末尾にある
typedef struct {
  vector<XZcoordinate> model_left;
  vector<XZcoordinate> model_right;
  int count_left;
  int count_right;
} NcmPoints;

// 負の遺産を継承するよ
class LiDAR2 :
    public PointCloudLiDAR
{
public:
  // 指定された方角を中心としたN(cm)のデータを返す
  NcmPoints getNcmPoints(const LiDAR_degree& direction, uint16_t range);

  LiDAR2() = default;

};
