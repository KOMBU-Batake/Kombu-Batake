#pragma once
#include "PointCloudLiDAR.h"

struct MAXandMIN {
  float leftMax;
  float leftMin;
  float rightMax;
  float rightMin;
};

// �^�񒆂�left�̖����ɂ���
struct NcmPoints {
  vector<XZcoordinate> model_left;
  vector<XZcoordinate> model_right;
  int count_left;
  int count_right;
};

struct StraightLine {
  float a;
  float b;
  float x_max;
  float x_min;
  float z_max;
  float z_min;

  StraightLine(XZcoordinate p1, XZcoordinate p2) {
    x_max = std::max(p1.x, p2.x);
    x_min = std::min(p1.x, p2.x);
    z_max = std::max(p1.z, p2.z);
    z_min = std::min(p1.z, p2.z);
    a = (p2.z - p1.z) / (p2.x - p1.x);
    b = p1.z - a * p1.x;
  }

  bool hasIntersection(const StraightLine& other) const {
    if (a == other.a) {
      if (b == other.b) return true;
			return false;
		}
    if ((other.a * x_max + other.b < z_max && other.a * x_min + other.b > z_min) ||
      (a * other.x_max + b < other.z_max && a * other.x_min + b > other.z_min)) {
      return true;
    }
    return false;
  }

  XZcoordinate getIntersectionPoint(const StraightLine& other) const {
    if (!hasIntersection(other)) {
      // �������Ȃ��ꍇ�� (NaN, NaN) ��Ԃ�
      return { std::nanf(""), std::nanf("") };
    }
    if (*this == other) {
      return { numeric_limits<float>::infinity(), numeric_limits<float>::infinity() };
    }

    // ��_���v�Z
    float x_intersection = (other.b - b) / (a - other.a);
    float z_intersection = a * x_intersection + b;
    return { x_intersection, z_intersection };
  }

  bool operator==(const StraightLine& other) const {
    return a == other.a && b == other.b;
  }

  StraightLine() = default;
};

// ���̈�Y���p�������
class LiDAR2 :
    public PointCloudLiDAR
{
public:
  // �w�肳�ꂽ���p�𒆐S�Ƃ���N(cm)�̃f�[�^��Ԃ�
  NcmPoints getNcmPoints(const LiDAR_degree& direction, float range);

  WallSet getWallType(const LiDAR_degree& direction);

  void update(GPSPosition goalPos) {
    PointCloudLiDAR::update(goalPos);
    for (int i = 0; i < 512; i++) {
			lines[i] = StraightLine(readPoint(i), readPoint(i+1));
		}
  }
private:
  // �w�肵�������ɂ���_
  int getCenterNum(LiDAR_degree direction, XZcoordinate centralPos = { 0,0 });
  // �ő�l�ƍŏ��l���擾����
  MAXandMIN getMAX_MIN(NcmPoints& pointsSet, LiDAR_degree direction);
  // �s���悭���W����]������
  void rotateToFront(vector<XZcoordinate>& points, LiDAR_degree direction);

  vector<XZcoordinate> getNcmPoints(XZcoordinate center, const LiDAR_degree& direction, float range);

  vector<StraightLine> getNcmLines(XZcoordinate center, const LiDAR_degree& direction, float range);

  void printLeftRight(const NcmPoints& pointsSet);
  
  XZcoordinate readPoint(int16_t num) {
    if (num < 0) num += 512;
    return pointCloud[num%512];
  }

  vector<StraightLine> lines = vector<StraightLine>(512);
};
