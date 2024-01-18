#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>

/* #easy�Ȃ킯�Ȃ����� */

using namespace webots;
using namespace std;

extern Lidar* centralLidar;
extern GyroZ gyro;

enum class LiDAR_degree {
	RELATIVE, // �@�̂��猩�����ΓI�Ȋp�x
	ABSOLUTE, // �t�B�[���h���猩����ΓI�Ȋp�x(���ʂƂ����Ă��悢)

	LEFT, // �@�̂̍��� �ȉ���
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
		// �p�x��0~360����0~511�ɂ��� ��?
		degree = degree * 512 / 360;
		degree = round(degree);
		if (degree == 512) degree = 0; // 512�Ԗڂ̒l�͑��݂��Ȃ�(0�Ɠ�����)
		int num = (int)degree;
		return rangeImage[num + 512 * 2];
	}

private:
	const float* rangeImage;

	void convertABSLOUTEtoRELATIVE(float& angle) { // ��Ίp�𑊑Ίp�ɕϊ�
		//double g_angle = 360 - gyro.getGyro();
		//angle = 360 - angle; // �����𔽓]
		//angle -= g_angle;
		angle = gyro.getGyro() - angle;
		if (angle < 0) angle += 360;
	}
};