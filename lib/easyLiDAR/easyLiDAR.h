#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include "../IMU/IMU.h"

/* #easyなわけないだろ */

using namespace webots;
using namespace std;

extern Lidar* centralLidar;
extern GyroZ gyro;

enum class LiDAR_degree {
	RELATIVE, // 機体から見た相対的な角度
	ABSOLUTE, // フィールドから見た絶対的な角度(方位とも言える)

	LEFT, // 機体の左側
	RIGHT,
	FRONT,
	BACK,

	FRONT_RIGHT,
	FRONT_LEFT,
};

typedef struct {
	float degree;
	float diff;
	float direction;
}directionInfo;

enum class DetailsofWall {
	noWALL,     // 0
	WALL,       // 1
	leftWALL,   // 2
	rightWALL,  // 3
	maybeWALL,  // 4
	maybeNOWALL,// 5
};

class LiDAR {
public:
	/* 値の更新 */
	void uodateLiDAR() {
		rangeImage = centralLidar->getRangeImage();
	}

	/* 上から3層目の値を返す */
	float getDistance(LiDAR_degree deg_option, float degree = 0) {
		if (deg_option != LiDAR_degree::RELATIVE) degree = deg_options(deg_option, degree).degree; // 諸々を相対角に変換
		// 角度を0~360から0~512にする は?
		degree = degree * 512 / 360;
		degree = round(degree);
		if (degree == 512) degree = 0; // 512番の値は存在しない(0と等しい)
		int num = (int)degree;
		return rangeImage[num + 1024] * 100;
	}

	DetailsofWall isWall(LiDAR_degree direction, float degree = 0) {

		if (direction == LiDAR_degree::RELATIVE || direction == LiDAR_degree::ABSOLUTE) { // そのまんま指定先の値で判断する。
			float distance = getDistance(direction,degree);
			if (distance > 6) return DetailsofWall::noWALL;
			else return DetailsofWall::WALL;
		}
		else {
			directionInfo disinfo = deg_options(direction, degree); // 全部相対角に変換

			float leftDistance, rightDistance;
			float leftDeg  = disinfo.degree - disinfo.diff; 
			float rightDeg = disinfo.degree + disinfo.diff;
			if (leftDeg < 0) leftDeg += 360;
			if (rightDeg > 360) rightDeg -= 360;
			rightDistance = getDistance(LiDAR_degree::RELATIVE, rightDeg);
			leftDistance = getDistance(LiDAR_degree::RELATIVE, leftDeg);

			if (direction == LiDAR_degree::FRONT_LEFT || direction == LiDAR_degree::FRONT_RIGHT) {
				if (rightDistance < disinfo.direction || leftDistance < disinfo.direction) return DetailsofWall::maybeWALL;
				else return DetailsofWall::maybeNOWALL;
			}
			else {
				if (rightDistance < disinfo.direction) {
					if (leftDistance < disinfo.direction) return DetailsofWall::WALL;
					else return DetailsofWall::rightWALL;
				}
				else if (leftDistance < disinfo.direction) {
					return DetailsofWall::leftWALL;
				}
				else return DetailsofWall::noWALL;
			}
		}
	}

	uint8_t convertCMtoTILE(float& distanceCM) {
		return (uint8_t)round((distanceCM - 5)/12);
	}

private:
	const float* rangeImage;

	void convertABSLOUTEtoRELATIVE(float& angle) { // 絶対角を相対角に変換
		//double g_angle = 360 - gyro.getGyro();
		//angle = 360 - angle; // 向きを反転
		//angle -= g_angle;
		angle = (float)gyro.getGyro() - angle;
		if (angle < 0) angle += 360;
	}

	directionInfo deg_options(LiDAR_degree deg_option, float degree = 0) {
		if (deg_option == LiDAR_degree::ABSOLUTE) {
			convertABSLOUTEtoRELATIVE(degree);
			return { degree,0 };
		}
		else if (deg_option == LiDAR_degree::LEFT) {
			return LEFT;
		}
		else if (deg_option == LiDAR_degree::RIGHT) {
			return RIGHT;
		}
		else if (deg_option == LiDAR_degree::BACK) {
			return BACK;
		}
		else if (deg_option == LiDAR_degree::FRONT) {
			return FRONT;
		}
		else if (deg_option == LiDAR_degree::FRONT_LEFT) {
			return FRONT_LEFT;
		}
		else if (deg_option == LiDAR_degree::FRONT_RIGHT) {
			return FRONT_RIGHT;
		}
		return { -1,-1,-1 }; // 知るか
	}

	directionInfo LEFT = { 270, 22.5, (float)6.7 };
	directionInfo RIGHT = { 90, 22.5, (float)6.7 };
	directionInfo BACK = { 180, 22.5, (float)6.7 };
	directionInfo FRONT = { 0, 22.5, (float)6.7 };
	directionInfo FRONT_LEFT = { (float)(270 + 63.435), 3.5, (float)13.416 };
	directionInfo FRONT_RIGHT = { (float)(90 - 63.435), 3.5, (float)13.416 };
};