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

class PointCloudLiDAR {
public:
	PointCloudLiDAR() {}

	void update(bool autoFix = true) {
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

	void update(float angle) {
		rangeImage = centralLidar->getRangeImage();
		converttoPointCloud();
		float gyro_angle = (float)gyro.getGyro();
		fixPointCloudAngle(angle,gyro_angle);
	}

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
	void fixPointCloudAngle(float angle_G_A, float gyro_angle) {
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
};