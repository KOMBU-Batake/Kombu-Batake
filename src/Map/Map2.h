#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <math.h>
#include <vector>
#include <algorithm>
#include <map>
#include <unordered_set>

#include "../../lib/GlobalPositioningSystem/GlobalPositioningSystem.h"
#include "../../lib/easyLiDAR/easyLiDAR.h"
#include "../../lib/IMU/IMU.h"
#include "../../lib/PointCloudLiDAR/PointCloudLiDAR.h"
#include "../../lib/ColorSensor/ColorSensor.h"
#include "./Map.h"

/* 提出用のマップを作りつつ、DFSにも利用するよ
 *
 * 座標の変数名の末尾に_AがついているものはABSOLUTEつまり絶対座標を
 * _RがついているものはRELATIVEつまり相対座標を表すよ
 *
 * ここでの絶対座標はmap_Aでのタイルの座標、相対座標はスタートタイルに対する座標を示す
 * なので絶対座標は負の値を取りえないが、相対座標は負の値を取ることがある
 *
 * リスト内でのx,zは基本的に絶対的な値だけを使う
 */

using namespace webots;
using namespace std;

extern Robot* robot;
extern GlobalPositioningSystem gps;

extern int timeStep;

class Map2 {
public:
	// ↓↓命と同じくらい大事↓↓
	vector<vector<string>> map_A = vector<vector<string>>(5, vector<string>(5, "-")); // とりあえず1x1のマップを作る
	// ↑↑命と同じくらい大事↑↑

	void addNorth(const int i = 1); // 北方向にタイルを追加
	void addSouth(const int i = 1); // 南方向にタイルを追加
	void addWest(const int j = 1); // 西方向にタイルを追加
	void addEast(const int j = 1); // 東方向にタイルを追加
private:

	MapAddress left_top_R = { 0, 0 }, right_bottom_R = { 0, 0 };
};