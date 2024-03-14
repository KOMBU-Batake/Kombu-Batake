#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <math.h>
#include <vector>
#include <algorithm>
#include <map>
#include <unordered_set>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>

#include "../../lib/ColorSensor/ColorSensor2.h"
#include "../../lib/IMU/IMU.h"
#include "../../lib/GlobalPositioningSystem/GlobalPositioningSystem.h"
#include "../../lib/Tank/Tank.h"
#include "../../lib/PointCloudLiDAR/PointCloudLiDAR.h"
#include "../Map/Map.h"
#include "../../lib/MyCam/MyCam.h"

using namespace webots;
using namespace std;

extern Robot* robot;
extern ColorSensor2 colorsensor;
extern GyroZ gyro;
extern GlobalPositioningSystem gps;
extern Tank tank;
extern Map mapper;
extern PointCloudLiDAR pcLiDAR;
extern Receiver* receiver;
extern Emitter* emitter;
extern MyCam myCam;

extern int timeStep;

enum class canGo {
	GO,
	NO,
	VISITED,
};

// 情報を知らせるニュースじゃなくてN(orth), E(ast), W(est), S(outh)の略だよ。まぁニュースも一緒だけど
enum class NEWS {
	NORTH,
	EAST,
	SOUTH,
	WEST,

	NO,
};

struct PotentialDirectionsOfTravel {
	canGo front;
	canGo back;
	canGo left;
	canGo right;
};

struct NEWSset {
	canGo north;
	canGo east;
	canGo south;
	canGo west;
};

struct DESelement {
	MapAddress target;
	NEWSset direction;
};

void DFS();

// マップデータとLiDARを基に進行方向を決定する エリア1飲みに対応
NEWS searchAround(double angle, int& tail, vector<MapAddress>& stack, bool& dontStack);

static void searchFront(PotentialDirectionsOfTravel& PDoT, WallSet& front_mp, const TileState& front_tile);
static void searchBack(PotentialDirectionsOfTravel& PDoT, WallSet& back_mp, const TileState& back_tile);
static void searchLeft(PotentialDirectionsOfTravel& PDoT, WallSet& left_mp, const TileState& left_tile);
static void searchRight(PotentialDirectionsOfTravel& PDoT, WallSet& right_mp, const TileState& right_tile);
static void HoleIsThere(const double& angle);
static void Area4IsThere(const double& angle, int tail, vector<MapAddress>& stack, bool& dontStack);
static bool condirtion_canGo(const WallSet& wallset);

// マップデータ提出できてエライ!!天才!!神!!
static void sendMap(vector<vector<string>>& map);