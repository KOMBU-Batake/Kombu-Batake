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

// 情報を知らせるニュースじゃなくてN(orth), E(ast), W(est), S(outh)の略だよ。まぁニュースも一緒だけど
enum class NEWS {
	NORTH,
	NORTH_EAST,
	EAST,
	SOUTH_EAST,
	SOUTH,
	SOUTH_WEST,
	WEST,
	NORTH_WEST,

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

struct stackDFS {
	MapAddress center = {0,0};
	vector<MapAddress> Go;
	vector<MapAddress> PartlyVisited;
	vector<MapAddress> diagonal;
};

void DFS();

canGo judgeCanGo(const vector<TileState>& tileState, const bool canGo);
bool addToStackDFS(vector<stackDFS>& stack, const double& angle, const canGo& front, const canGo& left, const canGo& right, const canGo& back, const canGo& front_left, const canGo& front_right);
MapAddress pickSatckDFS(vector<stackDFS>& stack);
void removeFromStackDFS(vector<stackDFS>& stack, const MapAddress& address);
void reallyAbleToGo(canGo& cango, const WallSet& wallset);
void deleteVisited(vector<stackDFS>& stack);

static void HoleIsThere(const double& angle);
static void Area4IsThere(const double& angle);
static bool condirtion_canGo(const WallSet& wallset);

// マップデータ提出できてエライ!!天才!!神!!
void sendMap(vector<vector<string>>& map);