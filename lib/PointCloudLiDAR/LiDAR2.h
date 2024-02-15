#pragma once
#include "PointCloudLiDAR.h"

struct MAXandMIN {
  float leftMax;
  float leftMin;
  float rightMax;
  float rightMin;
};

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
  NcmPoints getNcmPoints(const LiDAR_degree& direction, float range);

  WallSet getWallType(const LiDAR_degree& direction);
private:
  // 最大値と最小値を取得する
  MAXandMIN getMAX_MIN(NcmPoints& pointsSet, LiDAR_degree direction);
  void rotateToFront(NcmPoints& pointsSet, LiDAR_degree direction);

  void printLeftRight(const NcmPoints& pointsSet);
  
  XZcoordinate readPoint(int16_t num) {
    if (num < 0) num += 512;
    return pointCloud[num%512];
  }
};
