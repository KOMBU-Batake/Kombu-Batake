#pragma once
#include "PointCloudLiDAR.h"
#include <corecrt_math_defines.h>

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
  int count_left = 0;
  int count_right = 0;
  int centerNum = 0;
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

  void renewRangebyXmax(float new_x_max);
  void renewRangebyXmin(float new_x_min);
  void renewRangebyZmax(float new_z_max);
  void renewRangebyZmin(float new_z_min);
  bool hasIntersection(const StraightLine& other) const;
  XZcoordinate getIntersectionPoint(const StraightLine& other) const;

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
  vector<StraightLine> getNcmLines(XZcoordinate center, const LiDAR_degree& direction, float range);

  void test() {
    XZcoordinate p1 = { 1.0f, 2.0f };
    XZcoordinate p2 = { 3.0f, 4.0f };
    XZcoordinate p3 = { 5.0f, 6.0f };

    float angleABC = calculateAngle(p1, p2, p3);
    std::cout << "angle: " << angleABC << " do" << std::endl;
  }

private:
  // �w�肵�������ɂ���_
  int getCenterNum(LiDAR_degree direction, XZcoordinate centralPos = { 0,0 });
  // �ő�l�ƍŏ��l���擾����
  MAXandMIN getMAX_MIN(NcmPoints& pointsSet, LiDAR_degree direction);
  // �s���悭���W����]������
  void rotateToFront(vector<XZcoordinate>& points, LiDAR_degree direction);
  // �x�N�g���g���[�T�[�@�œ����_��T��
  vector<int> VectorTracer(NcmPoints& pointSet);

  vector<XZcoordinate> getNcmPoints(XZcoordinate center, const LiDAR_degree& direction, float range);

  WallType identifyLeft(vector<XZcoordinate>& leftPoints, const int& leftPointsCount);
  WallType identifyRight(vector<XZcoordinate>& rightPoints, const int& rightPointsCount);
  WallType identifyCenter(NcmPoints& pointSet, const WallSet& wallset);

  void printLeftRight(const NcmPoints& pointsSet);
  
  XZcoordinate readPoint(int16_t num) {
    if (num < 0) num += 512;
    return pointCloud[num%512];
  }

  float calculateAngle(const XZcoordinate& p1, const XZcoordinate& p2, const XZcoordinate& p3) {
    XZcoordinate ab, ac;
    ab.x = p1.x - p2.x;
    ab.z = p1.z - p2.z;
    ac.x = p3.x - p2.x;
    ac.z = p3.z - p2.z;

    float dotProductABAC = ab.x * ac.x + ab.z * ac.z;
    float lenAB = vectorLength(ab);
    float lenAC = vectorLength(ac);

    // cos(��)���v�Z
    float cosTheta = dotProductABAC / (lenAB * lenAC);
    if (cosTheta < -1.0F) cosTheta = -1.0F;
    if (cosTheta > 1.0F) cosTheta = 1.0F;

    return (float)acos(cosTheta) * 180.0F / 3.1415926535F; // �Ƃ����W�A������x�ɕϊ�
  }

  float vectorLength(const XZcoordinate& v) {
    return sqrt(v.x * v.x + v.z * v.z);
  }

  vector<StraightLine> lines = vector<StraightLine>(512);
};
