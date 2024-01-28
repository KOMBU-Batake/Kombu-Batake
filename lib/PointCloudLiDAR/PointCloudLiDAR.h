#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <math.h>
#include <vector>
#include <algorithm>
#include <map>
#include <unordered_set>

#include "../../lib/IMU/IMU.h"
#include "../../lib/GlobalPositioningSystem/GlobalPositioningSystem.h"
#include "../../lib/myMath/myMath.h"
#include "../../lib/easyLiDAR/easyLiDAR.h"

extern GyroZ gyro;
extern GlobalPositioningSystem gps;

extern Robot* robot;
extern Lidar* centralLidar;
extern int timeStep;

typedef struct {
	float x;
	float z;
}XZcoordinate;

enum class TagDegree { // 測定範囲のタグ
	leftLarge_rightLarge,
	leftLarge_rightSmall,
	leftSmall_rightLarge,
	leftSmall_rightSmall
};

enum class TagMinDistance { // 測定範囲の中で最も近い壁がどこにあるか
	one_point_five,
	two_point_five,
	three_point_five,
};

class pcDetails {
public:
	pcDetails(const LiDAR_degree lidarDeg, const TagDegree tagDeg, const TagMinDistance tagMinDis, const vector<float> _model){
		Degree = centralDegreeMap[lidarDeg];
		startDegree = Degree + startDegreeMap[tagDeg];
		endDegree = Degree + endDegreeMap[tagDeg];
		if (startDegree < 0) {
			startDegree += 512;
			flag0and512 = true;
		}
		minDistance = minDistanceMap[tagMinDis];
		model = _model;
	}

	vector<float> model;
	int startDegree = 0; // 0~512
	int Degree = 0; // 0~512
	int endDegree = 0; // 0~512
	bool flag0and512 = false; // 0度と512度を跨いだ範囲であるか
	float minDistance = 0; // 最も近い壁の距離の最大値
private:
	map<LiDAR_degree, int> centralDegreeMap = {
		{LiDAR_degree::FRONT,0},
		{LiDAR_degree::RIGHT,128},
		{LiDAR_degree::BACK, 256},
		{LiDAR_degree::LEFT, 384}
	};
	map<TagDegree, int> startDegreeMap = {
		{TagDegree::leftLarge_rightLarge, -36}, // -25(360) -35.555(512)
		{TagDegree::leftLarge_rightSmall, -36}, // -25(360)
		{TagDegree::leftSmall_rightLarge, -22}, // -15(360) -21.333(512)
		{TagDegree::leftSmall_rightSmall, -22}  // -15(360)
	};
	map<TagDegree, int> endDegreeMap = {
		{TagDegree::leftLarge_rightLarge, 36}, // 25(360)
		{TagDegree::leftLarge_rightSmall, 22}, // 15(360)
		{TagDegree::leftSmall_rightLarge, 36}, // 25(360)
		{TagDegree::leftSmall_rightSmall, 22}	// 15(360)
	};
	map<TagMinDistance,float> minDistanceMap = {
		{TagMinDistance::one_point_five, (float)9},
		{TagMinDistance::two_point_five, (float)15},
		{TagMinDistance::three_point_five, (float)21}
	};
};

class PointCloudLiDAR {
public:
	PointCloudLiDAR() {}

	// LiDARの値を更新する
	void update(const bool autoFix = true) {
		rangeImage = centralLidar->getRangeImage();
		converttoPointCloud();

		if (autoFix) {
			float gyro_angle = (float)gyro.getGyro();
			if (abs(gyro_angle - 90) < 5) {
				fixPointCloudAngle(90, gyro_angle);
			}
			else if (abs(gyro_angle - 180) < 5) {
				fixPointCloudAngle(180, gyro_angle);
			}
			else if (abs(gyro_angle - 270) < 5) {
				fixPointCloudAngle(270, gyro_angle);
			}
			else if ((gyro_angle <= 0 && gyro_angle < 5) || (gyro_angle > 355 && gyro_angle <= 360)) {
				fixPointCloudAngle(0, gyro_angle);
			}
		}
	}

	// 修正する角度を指定してLiDARの値を更新する
	void update(const float angle) {
		rangeImage = centralLidar->getRangeImage();
		converttoPointCloud();
		float gyro_angle = (float)gyro.getGyro();
		fixPointCloudAngle(angle,gyro_angle);
	}

	float getDistance(const float angle_R) {
		int angle512 = (int)round(angle_R * 512 / 360);
		return rangeImage[angle512 + 1024];
	}

	// LiDARの変換されたXZ平面上の点
	vector<XZcoordinate> pointCloud = vector<XZcoordinate>(512, { 0,0 });

private:
	const float* rangeImage = 0;
	MyMath myMath;

	void convertRELATIVEtoABSLOUTE(float& angle) { // 相対角を絶対角に変換
		//double g_angle = 360 - gyro.getGyro();
		//angle = 360 - angle; // 向きを反転
		//angle -= g_angle;
		angle = (float)gyro.getGyro() - angle;
		if (angle < 0) angle += 360;
	}

	void convertRELATIVEtoABSLOUTE(float& angle, const float& gyro) { // 相対角を絶対角に変換 IMUの値を受け取るver
		angle = gyro - angle;
		if (angle < 0) angle += 360;
	}

	void convertABSLOUTEtoRELATIVE(float& angle) { // 絶対角を相対角に変換
		//double g_angle = 360 - gyro.getGyro();
		//angle += g_angle;
		angle += (float)gyro.getGyro();
		if (angle > 360) angle -= 360;
	}

	void convertABSLOUTEtoRELATIVE(float& angle, const float& gyro) { // 絶対角を相対角に変換 IMUの値を受け取るver
		angle += gyro;
		if (angle > 360) angle -= 360;
	}

	// ポイントクラウドに変換
	void converttoPointCloud() {
		float tmpDistance = 0;
		for (int i = 0; i < 512; i++) {
			if (isinf(rangeImage[i + 1024])) {
				tmpDistance = (float)1.01;
			}
			else tmpDistance = rangeImage[i + 1024];
			tmpDistance *= 100; // m -> cm
			pointCloud[i].x = tmpDistance * myMath.sin[i];
			pointCloud[i].z = tmpDistance * myMath.cos[i];
			//cout << pointCloud[i].x << "," << pointCloud[i].z << endl;
		}
	}

	// 角度補正
	void fixPointCloudAngle(const float angle_G_A, const float& gyro_angle) {
		cout << "angle goal absolute" << angle_G_A << ", gyro angle" << gyro_angle << endl;
		//int da = (int)round((angle_G_A - gyro_angle) * 512 / 360); // 偏角を求める 0~360 -> 0~512 四捨五入
		float da = angle_G_A - gyro_angle;
		da *= -1; // 時計回りに変換
		cout << "da:" << da << endl;
		//if (da < 0) da += 512;
		//if (da > 512) da -= 512;
		// x=xcosθ−ysinθ
		// y=ycosθ+xsinθ
		da = da * (float)3.1415926535 / 180;
		cout << "da:" << da << endl;
		if (da != 0 && da != 512) {
			for (int i = 0; i < 512; i++) {
				pointCloud[i].x = pointCloud[i].x * (float)cos(da) - pointCloud[i].z * (float)sin(da);
				pointCloud[i].z = pointCloud[i].z * (float)cos(da) + pointCloud[i].x * (float)sin(da);
			}
		}
	}

	double pd_degrees_to_rad(const double degrees) {
		return degrees * 3.1415926535 / 180;
	}
};