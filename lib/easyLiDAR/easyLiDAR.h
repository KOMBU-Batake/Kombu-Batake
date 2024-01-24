#pragma once

#include <iostream>
#include <vector>
#include <algorithm>
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include "../IMU/IMU.h"
#include "../Tank/Tank.h"

/* #easy�Ȃ킯�Ȃ����� */

using namespace webots;
using namespace std;

extern Lidar* centralLidar;
extern GyroZ gyro;

enum class LiDAR_degree {
	RELATIVE, // �@�̂��猩�����ΓI�Ȋp�x
	ABSOLUTE, // �t�B�[���h���猩����ΓI�Ȋp�x(���ʂƂ�������)

	LEFT, // �@�̂̍���
	RIGHT,
	FRONT,
	BACK,

	LEFT_HALF,
	RIGHT_HALF,
	FRONT_HALF,
	BACK_HALF,

	FRONT_RIGHT,
	FRONT_LEFT,
};

enum class WallState { // ���ꂩ��q�킶��Ȃ����x���ő�����\��͖���
	noWALL,     // 0
	WALL,       // 1
	leftWALL,   // 2
	rightWALL,  // 3
	cneterWALL, // 4
	maybeWALL,  // 5
	maybeNOWALL,// 6
	unknown,    // 7
};

enum class relativeDirection{
	FRONT,
	RIGHT,
	BACK,
	LEFT,
};

class directionInfo {
public:
	float degree; // �ڕW�̊p�x
	float diff; // �O��͈̔�
	float distance; // ������傫����Εǃi�V�Ɣ��f
	int leftDeg512;
	int rightDeg512;
	int centerDeg512;
	bool flag0to360 = false;

	directionInfo(float deg, float dif, float dis) {
		degree = deg;
		diff = dif;
		distance = dis;
		float leftDeg = deg - dif;
		float rightDeg = deg + dif;
		if (leftDeg < 0) {
			flag0to360 = true;
			leftDeg += 360;
		}
		if (rightDeg >= 360) { // ����=�ɂ͈Ӗ������� 360��convert360to512�ɂ������0�ɕϊ�����邩�炻�̃t���O��\�ߗ��Ă�
			flag0to360 = true;
			rightDeg -= 360;
		}

		leftDeg512 = convert360to512(leftDeg);
		rightDeg512 = convert360to512(rightDeg);
		centerDeg512 = convert360to512(deg);
	}
private:
	int convert360to512(float degree) {
		degree = degree * 512 / 360;
		degree = round(degree);
		if (degree == 512) degree = 0; // 512�Ԃ̒l�͑��݂��Ȃ�(0�Ɠ�����)
		return (int)degree;
	}
};

class LiDAR {
public:
	/* �l�̍X�V */
	void updateLiDAR() {
		rangeImage = centralLidar->getRangeImage();
	}

	/* �ォ��3�w�ڂ̒l��Ԃ� */
	float getDistance(LiDAR_degree deg_option, float degree = 0) {
		if (deg_option != LiDAR_degree::RELATIVE) degree = deg_options(deg_option, degree).degree; // ���X�𑊑Ίp�ɕϊ�
		// �p�x��0~360����0~512�ɂ��� ��?
		degree = degree * 512 / 360;
		degree = round(degree);
		if (degree == 512) degree = 0; // 512�Ԃ̒l�͑��݂��Ȃ�(0�Ɠ�����)
		int num = (int)degree;
		return rangeImage[num + 1024] * 100;
	}

	WallState isWall(LiDAR_degree direction, float degree = 0);

	uint8_t convertCMtoTILE(float& distanceCM) {
		return (uint8_t)round((distanceCM - 5)/12);
	}

private:
	const float* rangeImage = 0;

	directionInfo LEFT = directionInfo(270, 22.5, (float)6.7);
	directionInfo RIGHT = directionInfo(90, 22.5, (float)6.7);
	directionInfo BACK = directionInfo(180, 22.5, (float)6.7);
	directionInfo FRONT = directionInfo(0, 22.5, (float)6.7);

	directionInfo LEFT_HALF = directionInfo(270, 14, (float)12.5);
	directionInfo RIGHT_HALF = directionInfo(90, 14, (float)12.5);
	directionInfo BACK_HALF = directionInfo(180, 14, (float)12.5);
	directionInfo FRONT_HALF = directionInfo(0, 14, (float)12.5);

	directionInfo FRONT_LEFT = directionInfo((float)(270 + 63.435), 3.5, 6.5);
	directionInfo FRONT_RIGHT = directionInfo((float)(90 - 63.435), 3.5, 6.5 );

	void convertABSLOUTEtoRELATIVE(float& angle) { // ��Ίp�𑊑Ίp�ɕϊ�
		//double g_angle = 360 - gyro.getGyro();
		//angle = 360 - angle; // �����𔽓]
		//angle -= g_angle;
		angle = (float)gyro.getGyro() - angle;
		if (angle < 0) angle += 360;
	}

	directionInfo deg_options(LiDAR_degree deg_option, float degree = 0) {
		if (deg_option == LiDAR_degree::ABSOLUTE) {
			convertABSLOUTEtoRELATIVE(degree);
			return directionInfo(degree, 0,6);
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
		else if (deg_option == LiDAR_degree::LEFT_HALF) {
			return LEFT_HALF;
		}
		else if (deg_option == LiDAR_degree::RIGHT_HALF) {
			return RIGHT_HALF;
		}
		else if (deg_option == LiDAR_degree::BACK_HALF) {
			return BACK_HALF;
		}
		else if (deg_option == LiDAR_degree::FRONT_HALF) {
			return FRONT_HALF;
		}
		return { -1,-1,-1 }; // �m�邩
	}

	int convert360to512(float degree){
		degree = degree * 512 / 360;
		degree = round(degree);
		if (degree == 512) degree = 0; // 512�Ԃ̒l�͑��݂��Ȃ�(0�Ɠ�����)
		return (int)degree;
	}

	static double pd_degrees_to_rad(double degrees) {
		return degrees * 3.1415926535 / 180;
	}
};