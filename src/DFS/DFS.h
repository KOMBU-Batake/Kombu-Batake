#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <math.h>

#include "../../lib/ColorSensor/ColorSensor.h"
#include "../../lib/IMU/IMU.h"
#include "../../lib/GlobalPositioningSystem/GlobalPositioningSystem.h"
#include "../../lib/ToF/ToF.h"
#include "../../lib/Tank/Tank.h"
#include "../../lib/easyLiDAR/easyLiDAR.h"
#include "../Map/Map.h"

using namespace webots;
using namespace std;

extern Robot* robot;
extern ColorSensor colorsensor;
extern GyroZ gyro;
extern GlobalPositioningSystem gps;
extern ToFSensor leftToF, rightToF;
extern Tank tank;
extern LiDAR lidar;
extern Map map;

extern int timeStep;

typedef struct{
	bool front;
	bool back;
	bool left;
	bool right;
} PotentialDirectionsOfTravel;

void DepthFirstSearch();