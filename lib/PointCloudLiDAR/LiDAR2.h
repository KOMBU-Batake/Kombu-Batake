#pragma once
#include "PointCloudLiDAR.h"
#include <corecrt_math_defines.h>

// �^�񒆂�left�̖����ɂ���
struct NcmPoints {
  vector<XZcoordinate> model_left;
  vector<XZcoordinate> model_right;
  int count_left = 0;
  int count_right = 0;
  int centerNum = 0;

  XZcoordinate read(int num) {
    if (num > count_left) {
      return model_right[num-count_left];
    }
    else return model_left[num];
  }

  bool isLeftEmpty() {
    auto closest = min_element(model_left.begin(), model_left.end(), [](XZcoordinate s1, XZcoordinate s2) { return (abs(s1.x + 3) < abs(s2.x + 3)); });
    return closest->z > 18;
	}

  bool isRightEmpty() {
    auto closest = min_element(model_right.begin(), model_right.end(), [](XZcoordinate s1, XZcoordinate s2) { return (abs(s1.x - 3) < abs(s2.x - 3)); });
    return closest->z > 18;
	}
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
  struct XZrange {
    XZrange(const float xmax, const float xmin, const float zmax, const float zmin) {
      x_max = xmax;
      x_min = xmin;
      z_max = zmax;
      z_min = zmin;
      if (x_min > x_max) {
        x_min = xmax;
        x_max = xmin;
      }
      if (z_min > z_max) {
        z_min = zmax;
        z_max = zmin;
      }
    }

    bool isIncluding(XZcoordinate p){
      return (p.x <= x_max && p.x >= x_min && p.z <= z_max && p.z >= z_min);
    }

    float x_max;
    float x_min;
    float z_max;
    float z_min;

    XZrange() = default;
  };

  // �w�肵�������ɂ���_
  int getCenterNum(LiDAR_degree direction, XZcoordinate centralPos = { 0,0 });
  // �s���悭���W����]������
  void rotateToFront(vector<XZcoordinate>& points, LiDAR_degree direction);
  // �x�N�g���g���[�T�[�@�œ����_��T��
  vector<int> VectorTracer(NcmPoints& pointSet, bool display = false);

  vector<XZcoordinate> getNcmPoints(XZcoordinate center, const LiDAR_degree& direction, float range);

  WallType identifyLeft(NcmPoints& pointSet, vector<int>& featurePoints);
  WallType identifyRight(NcmPoints& pointSet, vector<int>& featurePoints);
  WallType identifyCenter(NcmPoints& pointSet, const WallSet& wallset, vector<int>& featurePoints);

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

  float Variance(vector<XZcoordinate>::iterator startIt, vector<XZcoordinate>::iterator endIt, int start, int end){
    // Z�������̕���
    float sumZ = 0.0f;
    for (auto it = startIt; it != endIt + 1; ++it) sumZ += it->z;
    // 2��̕���
    float sumZpow2 = 0.0f;
    for (auto it = startIt; it != endIt + 1; ++it) sumZpow2 += (float)pow(it->z,2);
    // ���U
    float variance = (float)(sumZpow2 / (end - start + 1) - pow(sumZ / (end - start + 1), 2));
    //std::cout << "variance: " << variance << endl;
    return variance;
  }

  WallType identifyCurve(vector<XZcoordinate>::iterator startIt, vector<XZcoordinate>::iterator endIt) {
    // �Ȑ�
    if (startIt->z > endIt->z) // ���ی� or ��O�ی�
    {
      float leftDiff = (startIt->z - (startIt + 4)->z) / (startIt->x - (startIt + 4)->x); // ���[�̔���
      float rightDiff = (endIt->z - (endIt - 4)->z) / (endIt->x - (endIt - 4)->x); // �E�[�̔���
      std::cout << "first or third, " << leftDiff << " " << rightDiff << endl;
      if (abs(leftDiff) < abs(rightDiff)) return WallType::type1;
      else return WallType::type3;
    }
    else // ���ی� or ��l�ی�
    {
      float leftDiff = ((startIt + 4)->z - startIt->z) / ((startIt + 4)->x - startIt->x); // ���[�̔���
      float rightDiff = ((endIt - 4)->z - endIt->z) / ((endIt - 4)->x - endIt->x); // �E�[�̔���
      std::cout << "second or fourth, " << leftDiff << " " << rightDiff << endl;
      if (abs(leftDiff) > abs(rightDiff)) return WallType::type4; // ���ی�
      else return WallType::type2; // ��l�ی�
    }
  }

  vector<StraightLine> lines = vector<StraightLine>(512);
};
