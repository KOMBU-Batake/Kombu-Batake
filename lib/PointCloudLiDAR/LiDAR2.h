#pragma once
#include "PointCloudLiDAR.h"

struct MAXandMIN {
  float leftMax;
  float leftMin;
  float rightMax;
  float rightMin;
};

// 真ん中はleftの末尾にある
struct NcmPoints {
  vector<XZcoordinate> model_left;
  vector<XZcoordinate> model_right;
  int count_left;
  int count_right;
};

struct StraightLine {
  StraightLine(XZcoordinate p1, XZcoordinate p2) {
    x_max = max(p1.x, p2.x);
    x_min = min(p1.x, p2.x);
    z_max = max(p1.z, p2.z);
    z_min = min(p1.z, p2.z);
    a = (p2.z - p1.z) / (p2.x - p1.x);
    b = p1.z - a * p1.x;
  }
  float a;
  float b;
  float x_max;
  float x_min;
  float z_max;
  float z_min;

  StraightLine() = default;
};

// 負の遺産を継承するよ
class LiDAR2 :
    public PointCloudLiDAR
{
public:
  // 指定された方角を中心としたN(cm)のデータを返す
  NcmPoints getNcmPoints(const LiDAR_degree& direction, float range);

  WallSet getWallType(const LiDAR_degree& direction);

  void update(GPSPosition goalPos) {
    PointCloudLiDAR::update(goalPos);
    for (int i = 0; i < 512; i++) {
			lines[i] = StraightLine(readPoint(i), readPoint(i+1));
		}
  }
private:
  // 最大値と最小値を取得する
  MAXandMIN getMAX_MIN(NcmPoints& pointsSet, LiDAR_degree direction);
  // 都合よく座標を回転させる
  void rotateToFront(NcmPoints& pointsSet, LiDAR_degree direction);

  void printLeftRight(const NcmPoints& pointsSet);
  
  XZcoordinate readPoint(int16_t num) {
    if (num < 0) num += 512;
    return pointCloud[num%512];
  }

  vector<StraightLine> lines = vector<StraightLine>(512);
};
