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
	void setStartAngle() { // IMUをenableした後に呼び出すこと
		startAngle = getGyro();
	}
	double startAngle = 0;
};

