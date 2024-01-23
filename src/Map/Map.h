#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <math.h>
#include <vector>
#include <algorithm>
//#include <deque>
#include <map>

#include "../../lib/GlobalPositioningSystem/GlobalPositioningSystem.h"

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

typedef struct {
	int x;
	int z;
}MapAddress;

enum class TileState {
	OTHER = 0,
	WALL = 1,
	HOLE = 2,
	SWAMP = 3,
	CHECKPOINT = 4,
	START = 5,
	AREA1to2 = 6,
	AREA2to3 = 7,
	AREA3to4 = 8,
	AREA1to4 = 9,
	UNKNOWN,
};

enum class VictimState {
	H, // 重度の被災者
	S, // 軽症の被災者
	U, // 無傷の被災者
	FlammableGas, // 可燃性ガス: F
	Poison, // 毒: P
	Corrosive, // 腐食性: C
	OrganicPeroxide, // 有機過酸化物: O
};

class Map {
public:
	// ↓↓命と同じくらい大事↓↓
	vector<vector<string>> map_A = vector<vector<string>>(5, vector<string>(5, "-")); // とりあえず1x1のマップを作る
	// ↑↑命と同じくらい大事↑↑

	Map() {
		markTileAs({ 0, 0 }, getTileName(TileState::START));
	}

	void markTileAs(MapAddress add_R, string tile) {
		MapAddress add_L = convertRtoListPoint(add_R);
		map_A[add_L.z - 1][add_L.x - 1] = tile;
		map_A[add_L.z - 1][add_L.x + 1] = tile;
		map_A[add_L.z + 1][add_L.x - 1] = tile;
		map_A[add_L.z + 1][add_L.x + 1] = tile;
	}

	void addNorth() { // 上に追加
		// map_Aの下に1行追加
		map_A.push_back(vector<string>(map_A.size(), "-"));
		// map_Aの要素を下に1つずらす
		rotate(map_A.begin(), map_A.begin() + 1, map_A.end());
		startTile_A.z++;
		currentTile_A.z++;
	}

	void addSouth() { // 下に追加
		// map_Aの下に1行追加
		map_A.push_back(vector<string>(map_A.size(), "-"));
	}

	void addWest() { // 左に追加
		// 既存の行に新しい列を追加
		for (int i = 0; i < map_A.size(); ++i) {
			map_A[i].push_back("-");
		}
		// 全体の要素を右に1つずらす
		for (int i = 0; i < map_A.size(); ++i) {
			rotate(map_A[i].begin(), map_A[i].begin() + 1, map_A[i].end());
		}
		startTile_A.x++;
		currentTile_A.x++;
	}

	void addEast() { // 右に追加
		// 既存の行に新しい列を追加
		for (int i = 0; i < map_A.size(); ++i) {
			map_A[i].push_back("-");
		}
	}

	string getTileName(TileState state) {
		return TileStateMap[state];
	}

	string getVictimName(VictimState state) {
		return VictimStateMap[state];
	}
private: // ************************************************************************************
	/* なんだコレ */
	int convertTiletoList(int tile) {
		return tile * 4;
	}

	/* 絶対座標を相対座標に変換 */
	MapAddress convertAtoR(const MapAddress& addr_A) {
		return { addr_A.x - startTile_A.x, addr_A.z - startTile_A.z };
	}

	/* 相対座標を絶対座標に変換 */
	MapAddress convertRtoA(const MapAddress& addr_R) {
		return { addr_R.x + startTile_A.x, addr_R.z + startTile_A.z };
	}

	/* タイル座標からリストのタイル中心の座標に変換 */
	MapAddress convertAtoListPoint(const MapAddress& addr_A) {
		return { addr_A.x * 4 + 3, addr_A.z * 4 + 3 };
	}

	/* リストのタイル中心の座標からタイル座標に変換 */
	MapAddress convertListPointtoA(const MapAddress& addr_list) {
		return { (addr_list.x - 3) / 4, (addr_list.z - 3) / 4 };
	}

	/* 相対座標からリストのタイル中心の座標に変換 */
	MapAddress convertRtoListPoint(const MapAddress& addr_R) {
		return convertAtoListPoint(convertRtoA(addr_R));
	}

	/* リストのタイル中心の座標から相対座標に変換 */
	MapAddress convertListPointtoR(const MapAddress& addr_list) {
		return convertAtoR(convertListPointtoA(addr_list));
	}

	MapAddress startTile_A = { 0, 0 }, startTile_R = { 0, 0 };
	MapAddress currentTile_A = { 0, 0 }, currentTile_R = { 0, 0 };

	std::map<TileState, string> TileStateMap = {
		{TileState::OTHER, "0"},
		{TileState::WALL, "1"},
		{TileState::HOLE, "2"},
		{TileState::SWAMP, "3"},
		{TileState::CHECKPOINT, "4"},
		{TileState::START, "5"},
		{TileState::AREA1to2, "6"},
		{TileState::AREA2to3, "7"},
		{TileState::AREA3to4, "8"},
		{TileState::AREA1to4, "9"},
		{TileState::UNKNOWN, "-"},	
	};

	std::map<VictimState, string> VictimStateMap = {
		{VictimState::H, "H"},
		{VictimState::S, "S"},
		{VictimState::U, "U"},
		{VictimState::FlammableGas, "F"},
		{VictimState::Poison, "P"},
		{VictimState::Corrosive, "C"},
		{VictimState::OrganicPeroxide, "O"},
	};
};