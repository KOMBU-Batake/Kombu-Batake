#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>

#include "../IMU/IMU.h"

using namespace webots;
using namespace std;

extern GPS* gpsXZ;
extern GyroZ gyro;

typedef struct {
	double x;
	double z;
} GPSPosition;

//extern GPSPosition startPos, expectedPos, last_expectedPos;

class GlobalPositioningSystem
{
public:
	GPSPosition startPos, expectedPos, last_expectedPos;

	/* ’PˆÊ‚Ícm */ 
	GPSPosition getPosition() {
		const double* gpsValues = gpsXZ->getValues();
		GPSPosition gpsPosition;
		double angle = gyro.getGyroRad();
		gpsPosition.x = gpsValues[0] * 100 - 3.7 * sin(angle);
		gpsPosition.z = gpsValues[2] * 100 - 3.7 * cos(angle);
		return gpsPosition;
	}

	GPSPosition getPositionRAW() {
		const double* gpsValues = gpsXZ->getValues();
		GPSPosition gpsPosition;
		gpsPosition.x = gpsValues[0] * 100;
		gpsPosition.z = gpsValues[2] * 100;
		return gpsPosition;
	}

	GPSPosition filter(GPSPosition gpsPosition) {
		double angle = gyro.getGyroRad();
		gpsPosition.x = gpsPosition.x - 3.7 * sin(angle);
		gpsPosition.z = gpsPosition.z - 3.7 * cos(angle);
		return gpsPosition;
	}

	GPSPosition moveTiles(int x, int z) {
		last_expectedPos = expectedPos;
		expectedPos.x += x * 12;
		expectedPos.z += z * 12;
		return expectedPos;
	}

	bool comparePositions(bool ifchange = false) {
		GPSPosition currentPos = getPosition();
		double dis = sqrt(pow(currentPos.x - expectedPos.x, 2) + pow(currentPos.z - expectedPos.z, 2));
		if (dis > 2) {
			if (ifchange) expectedPos = currentPos;
			return false;
		}
		return true;
	}

	void recoedStartPosition() {
		startPos = getPosition();
		expectedPos = startPos;
		last_expectedPos = startPos;
		return;
	}
};
