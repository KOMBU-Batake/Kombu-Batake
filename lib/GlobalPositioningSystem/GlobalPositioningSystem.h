#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>

using namespace webots;
using namespace std;

extern GPS* gpsXZ;

typedef struct {
	double x;
	double z;
} GPSPosition;

class GlobalPositioningSystem
{
public:
	/* ’PˆÊ‚Ícm */ 
	GPSPosition getPosition() {
		const double* gpsValues = gpsXZ->getValues();
		GPSPosition gpsPosition;
		gpsPosition.x = gpsValues[0] * 100;
		gpsPosition.z = gpsValues[2] * 100;
		return gpsPosition;
	}
};

