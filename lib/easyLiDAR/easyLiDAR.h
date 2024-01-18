#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>

/* #easyなわけないだろ */

using namespace webots;
using namespace std;

extern Lidar* centralLidar;
extern GyroZ gyro;

enum class LiDAR_degree {
	RELATIVE, // 機体から見た相対的な角度
	ABSOLUTE, // フィールドから見た絶対的な角度(方位といってもよい)

	LEFT, // 機体の左側 以下略
	RIGHT,
	FRONT,
	BACK,
};

class LiDAR {
public:
	void uodateLiDAR() {
		rangeImage = centralLidar->getRangeImage();
	}

	float getDistance(LiDAR_degree deg_option = LiDAR_degree::RELATIVE, float degree = 0) {
		if (deg_option == LiDAR_degree::ABSOLUTE) convertABSLOUTEtoRELATIVE(degree);
		if (deg_option == LiDAR_degree::LEFT) degree = 270;
		if (deg_option == LiDAR_degree::RIGHT) degree = 90;
		if (deg_option == LiDAR_degree::BACK) degree = 180;
		if (deg_option == LiDAR_degree::FRONT) degree = 0;
		// 角度を0~360から0~511にする は?
		degree = degree * 512 / 360;
		degree = round(degree);
		if (degree == 512) degree = 0; // 512番目の値は存在しない(0と等しい)
		int num = (int)degree;
		return rangeImage[num + 512 * 2];
	}

private:
	const float* rangeImage;

	void convertABSLOUTEtoRELATIVE(float& angle) { // 絶対角を相対角に変換
		//double g_angle = 360 - gyro.getGyro();
		//angle = 360 - angle; // 向きを反転
		//angle -= g_angle;
		angle = gyro.getGyro() - angle;
		if (angle < 0) angle += 360;
	}
};