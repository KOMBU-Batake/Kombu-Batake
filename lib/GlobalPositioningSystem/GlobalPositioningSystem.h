#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>

using namespace webots;
using namespace std;

extern GPS* gpsXZ;

typedef struct {
	double x;
	double y;
	double z;
} GPSPosition;

class GlobalPositioningSystem
{
public:
	// �P�ʂ�cm
	GPSPosition getPosition() {
		const double* gpsValues = gpsXZ->getValues();
		GPSPosition gpsPosition;
		gpsPosition.x = gpsValues[0] * 100;
		gpsPosition.y = gpsValues[1] * 100; // ���Ӗ� ���\�[�X�̖���
		gpsPosition.z = gpsValues[2] * 100;
		return gpsPosition;
	}
};

