#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/InertialUnit.hpp>

using namespace webots;
using namespace std;

extern InertialUnit* IMU;

class GyroZ
{
public:
	/* �P�ʂ͓x���@ 
	 * ����������_�ɔ����v���𐳂Ƃ��� 
	 * �͈͂�0~360 
	 * Z���̒l������Ԃ� 
	 */
	double getGyro() {
		return IMU->getRollPitchYaw()[2] * 180 / 3.141592653589 + 180;
	}
	void setStartAngle() { // IMU��enable������ɌĂяo������
		startAngle = getGyro();
	}
	double startAngle = 0;
};

