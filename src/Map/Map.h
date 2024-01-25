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

/* ��o�p�̃}�b�v�����ADFS�ɂ����p�����
 * 
 * ���W�̕ϐ����̖�����_A�����Ă�����̂�ABSOLUTE�܂��΍��W��
 * _R�����Ă�����̂�RELATIVE�܂葊�΍��W��\����
 * 
 * �����ł̐�΍��W��map_A�ł̃^�C���̍��W�A���΍��W�̓X�^�[�g�^�C���ɑ΂�����W������
 * �Ȃ̂Ő�΍��W�͕��̒l����肦�Ȃ����A���΍��W�͕��̒l����邱�Ƃ�����
 * 
 * ���X�g���ł�x,z�͊�{�I�ɐ�ΓI�Ȓl�������g��
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
	H, // �d�x�̔�Ў�
	S, // �y�ǂ̔�Ў�
	U, // �����̔�Ў�
	FlammableGas, // �R���K�X: F
	Poison, // ��: P
	Corrosive, // ���H��: C
	OrganicPeroxide, // �L�@�ߎ_����: O
};

class Map {
public:
	// �������Ɠ������炢�厖����
	vector<vector<string>> map_A = vector<vector<string>>(5, vector<string>(5, "-")); // �Ƃ肠����1x1�̃}�b�v�����
	// �������Ɠ������炢�厖����

	Map() {
		markTileAs({ 0, 0 }, TileState::START);
	}

	// �^�C���̒��S�ɓ��B�����珉�߂ɌĂ�
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

	/* ���Ɏg���˂񂱂�{ */
	void markAroundTile(TileState front, TileState back, TileState left, TileState right, float angle = -1);

	TileState getTileState(MapAddress addr_R);

	// ���̕ǂ܂ł͒��ׂĂȂ�
	void getAroundTileState(MapAddress addr_R, TileState& front, TileState& back, TileState& left, TileState& right, double angle = -1);

	// �Ƃ肠�����^�C�����ׂ��悤�ȋȐ��Ƃ��̕ςȕǂ͂Ȃ����̂Ƃ��čl����B ToDo: LiDAR�Ń��[�N���b�h�����̔�r����������
	void markAroundWall(WallState NorthWall, WallState SouthWall, WallState WestWall, WallState EastWall);

	// ������k�̕ǂ����Ă���
	void markNorthWall(MapAddress addr_R, WallState wall);
	void markSouthWall(MapAddress addr_R, WallState wall);
	void markWestWall(MapAddress addr_R, WallState wall);
	void markEastWall(MapAddress addr_R, WallState wall);

	// �ǂ̗L�������𒲂ׂ� ��̓I�Ȏ�ނ͔��ʂ��Ȃ�
	void getAroundWallState(const MapAddress& addr_R, WallState& frontWall, WallState& backWall, WallState& leftWall, WallState& rightWall, double angle = -1);

	// �p�̏���
	void edge(MapAddress add_R, MapAddress add_L = { -1,-1 });

	
	void addNorth(const int i = 1); // �k�����Ƀ^�C����ǉ�
	void addSouth(const int i = 1); // ������Ƀ^�C����ǉ�
	void addWest(const int j = 1); // �������Ƀ^�C����ǉ�
	void addEast(const int j = 1); // �������Ƀ^�C����ǉ�

	// �}�b�v�S�̂�\������
	void printMap();

	// �Ō��"-"��"0"�ɕύX���ď����ł��|�C���g���҂����Ƒ��~��
	void replaceLineTo0();

	MapAddress startTile_A = { 0, 0 }, startTile_R = { 0, 0 };
	MapAddress currentTile_A = { 0, 0 }, currentTile_R = { 0, 0 };
private: // ************************************************************************************
	/* �Ȃ񂾃R�� */
	int convertTiletoList(int tile) {
		return tile * 4;
	}

	/* ��΍��W�𑊑΍��W�ɕϊ� */
	MapAddress convertAtoR(const MapAddress& addr_A) {
		return { addr_A.x - startTile_A.x, addr_A.z - startTile_A.z };
	}

	/* ���΍��W���΍��W�ɕϊ� */
	MapAddress convertRtoA(const MapAddress& addr_R) {
		return { addr_R.x + startTile_A.x, addr_R.z + startTile_A.z };
	}

	/* �^�C�����W���烊�X�g�̃^�C�����S�̍��W�ɕϊ� */
	MapAddress convertAtoListPoint(const MapAddress& addr_A) {
		return { addr_A.x * 4 + 2, addr_A.z * 4 + 2 };
	}

	/* ���X�g�̃^�C�����S�̍��W����^�C�����W�ɕϊ� */
	MapAddress convertListPointtoA(const MapAddress& addr_list) {
		return { (addr_list.x - 2) / 4, (addr_list.z - 2) / 4 };
	}

	/* ���΍��W���烊�X�g�̃^�C�����S�̍��W�ɕϊ� */
	MapAddress convertRtoListPoint(const MapAddress& addr_R) {
		return convertAtoListPoint(convertRtoA(addr_R));
	}

	/* ���X�g�̃^�C�����S�̍��W���瑊�΍��W�ɕϊ� */
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
		{WallState::noWALL,{"0","0","0"}}, // left -> center -> right �̏�
		{WallState::WALL,{"1","1","1"}},
		{WallState::leftWALL,{"1","1","0"}},
		{WallState::rightWALL,{"0","1","1"}},
		{WallState::cneterWALL,{"0","1","0"}},
		//{WallState::unknown,{"-","-","-"}},
	};

	std::unordered_set<string> WallAndVictim = { "1","H","S","U","F","P","C","O" };
	MapAddress left_top_R = { 0, 0 }, right_bottom_R = { 0, 0 };
};