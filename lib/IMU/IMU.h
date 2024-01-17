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
	/* 単位は度数法 
	 * 南向きを原点に反時計回りを正とする 
	 * 範囲は0~360 
	 * Z軸の値だけを返す 
	 */
	double getGyro() {
		return IMU->getRollPitchYaw()[2] * 180 / 3.141592653589 + 180;
	}

	double getGyroRad() {
		return IMU->getRollPitchYaw()[2] + 3.141592653589;
	}

	void setStartAngle() { // IMUをenableした後に呼び出すこと
		startAngle = getGyro();
	}
	double startAngle = 0;

	double getGyroEAST() {
		double angle = IMU->getRollPitchYaw()[2] * 180 / 3.141592653589 + 180;
		pd_angle(angle);
		return angle;
	}

	static void pd_angle(double& angle) {
		if (angle < 90) {
			angle += 270;
		}
		else angle -= 90;
	}
};

