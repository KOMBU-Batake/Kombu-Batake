#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <math.h>
#include <vector>
#include <algorithm>
#include <map>
#include <unordered_set>

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
extern Map mapper;

extern int timeStep;

typedef struct{
	WallState front;
	WallState back;
	WallState left;
	WallState right;
} PotentialDirectionsOfTravel;

void DFS();

// マップデータとLiDARを基に進行方向を決定する エリア1飲みに対応
static LiDAR_degree searchAround(double angle);

static void searchFront(PotentialDirectionsOfTravel& PDoT, WallState& front_mp, WallState& front_li, TileState& front_tile, double& angle);
static void searchBack(PotentialDirectionsOfTravel& PDoT, WallState& back_mp, WallState& back_li, TileState& back_tile, double& angle);
static void searchLeft(PotentialDirectionsOfTravel& PDoT, WallState& left_mp, WallState& left_li, TileState& left_tile, double& angle);
static void searchRight(PotentialDirectionsOfTravel& PDoT, WallState& right_mp, WallState& right_li, TileState& right_tile, double& angle);