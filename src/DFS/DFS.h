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

// ����m�点��j���[�X����Ȃ���N(orth), E(ast), W(est), S(outh)�̗�����B�܂��j���[�X���ꏏ������
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

// �}�b�v�f�[�^��LiDAR����ɐi�s���������肷�� �G���A1���݂ɑΉ�
NEWS searchAround(double angle, int& tail, vector<MapAddress>& stack, bool& dontStack);

static void searchFront(PotentialDirectionsOfTravel& PDoT, WallSet& front_mp, const TileState& front_tile);
static void searchBack(PotentialDirectionsOfTravel& PDoT, WallSet& back_mp, const TileState& back_tile);
static void searchLeft(PotentialDirectionsOfTravel& PDoT, WallSet& left_mp, const TileState& left_tile);
static void searchRight(PotentialDirectionsOfTravel& PDoT, WallSet& right_mp, const TileState& right_tile);
static void HoleIsThere(const double& angle);
static void Area4IsThere(const double& angle, int tail, vector<MapAddress>& stack, bool& dontStack);
static bool condirtion_canGo(const WallSet& wallset);

// �}�b�v�f�[�^��o�ł��ăG���C!!�V��!!�_!!
static void sendMap(vector<vector<string>>& map);