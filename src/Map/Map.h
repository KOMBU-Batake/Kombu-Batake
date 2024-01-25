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
	UNKNOWN, // = "-
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
		markTileAs({ 0, 0 }, TileState::START);
	}

	// タイルの中心に到達したら初めに呼ぶ
	void updatePostion(int dx_R, int dz_R);

	void markTileAs(MapAddress add_R, TileState tilestate) {
		if (add_R.x < left_top_R.x) {
			addWest(left_top_R.x - add_R.x);
		} else if (add_R.x > right_bottom_R.x) {
			addEast(add_R.x - right_bottom_R.x);
		}
		if (add_R.z < left_top_R.z) {
			addNorth(left_top_R.z - add_R.z);
		}else if (add_R.z > right_bottom_R.z) {
			addSouth(add_R.z - right_bottom_R.z);
		}
		MapAddress add_L = convertRtoListPoint(add_R);
		string tile = TileStateMap[tilestate];
		map_A[add_L.z - 1][add_L.x - 1] = tile;
		map_A[add_L.z - 1][add_L.x + 1] = tile;
		map_A[add_L.z + 1][add_L.x - 1] = tile;
		map_A[add_L.z + 1][add_L.x + 1] = tile;

		map_A[add_L.z - 1][add_L.x] = "0";
		map_A[add_L.z + 1][add_L.x] = "0";
		map_A[add_L.z][add_L.x - 1] = "0";
		map_A[add_L.z][add_L.x + 1] = "0";
		map_A[add_L.z][add_L.x] = "0";
	}

	/* 何に使うねんこれ怒 */
	void markAroundTile(TileState front, TileState back, TileState left, TileState right, float angle = -1);

	TileState getTileState(MapAddress addr_R);

	// 中の壁までは調べてない
	void getAroundTileState(MapAddress addr_R, TileState& front, TileState& back, TileState& left, TileState& right, double angle = -1);

	// とりあえずタイルを跨ぐような曲線とかの変な壁はないものとして考える。 ToDo: LiDARでユークリッド距離の比較を実装する
	void markAroundWall(WallState NorthWall, WallState SouthWall, WallState WestWall, WallState EastWall);

	// 東西南北の壁をあてがう
	void markNorthWall(MapAddress addr_R, WallState wall);
	void markSouthWall(MapAddress addr_R, WallState wall);
	void markWestWall(MapAddress addr_R, WallState wall);
	void markEastWall(MapAddress addr_R, WallState wall);

	// 壁の有無だけを調べる 具体的な種類は判別しない
	void getAroundWallState(const MapAddress& addr_R, WallState& frontWall, WallState& backWall, WallState& leftWall, WallState& rightWall, double angle = -1);

	// 角の処理
	void edge(MapAddress add_R, MapAddress add_L = { -1,-1 });

	
	void addNorth(const int i = 1); // 北方向にタイルを追加
	void addSouth(const int i = 1); // 南方向にタイルを追加
	void addWest(const int j = 1); // 西方向にタイルを追加
	void addEast(const int j = 1); // 東方向にタイルを追加

	// マップ全体を表示する
	void printMap();

	// 最後に"-"を"0"に変更して少しでもポイントを稼ごうと足掻く
	void replaceLineTo0();

	MapAddress startTile_A = { 0, 0 }, startTile_R = { 0, 0 };
	MapAddress currentTile_A = { 0, 0 }, currentTile_R = { 0, 0 };
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
		return { addr_A.x * 4 + 2, addr_A.z * 4 + 2 };
	}

	/* リストのタイル中心の座標からタイル座標に変換 */
	MapAddress convertListPointtoA(const MapAddress& addr_list) {
		return { (addr_list.x - 2) / 4, (addr_list.z - 2) / 4 };
	}

	/* 相対座標からリストのタイル中心の座標に変換 */
	MapAddress convertRtoListPoint(const MapAddress& addr_R) {
		return convertAtoListPoint(convertRtoA(addr_R));
	}

	/* リストのタイル中心の座標から相対座標に変換 */
	MapAddress convertListPointtoR(const MapAddress& addr_list) {
		return convertAtoR(convertListPointtoA(addr_list));
	}

	bool existTile_R(const MapAddress& addr_R) {
		if (addr_R.x < left_top_R.x) {
			return false;
		}
		else if (addr_R.x > right_bottom_R.x) {
			return false;
		}
		if (addr_R.z < left_top_R.z) {
			return false;
		}
		else if (addr_R.z > right_bottom_R.z) {
			return false;
		}
		return true;
	}

	WallState condition_getAroundWallState(int x, int z) {
		if (WallAndVictim.find(map_A[z][x]) != WallAndVictim.end()) 
		{
			return WallState::WALL;
		} 
		else if (map_A[z][x] == "-") 
		{
			return WallState::unknown;
		}
		else {
			return WallState::noWALL;
		}
	}

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

	std::map<WallState, vector<string>> WallStateMap = {
		{WallState::noWALL,{"0","0","0"}}, // left -> center -> right の順
		{WallState::WALL,{"1","1","1"}},
		{WallState::leftWALL,{"1","1","0"}},
		{WallState::rightWALL,{"0","1","1"}},
		{WallState::cneterWALL,{"0","1","0"}},
		//{WallState::unknown,{"-","-","-"}},
	};

	std::unordered_set<string> WallAndVictim = { "1","H","S","U","F","P","C","O" };
	MapAddress left_top_R = { 0, 0 }, right_bottom_R = { 0, 0 };
};