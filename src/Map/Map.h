#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <math.h>
#include <vector>
#include <algorithm>
#include <map>
#include <unordered_set>

#include "../../lib/GlobalPositioningSystem/GlobalPositioningSystem.h"
#include "../../lib/IMU/IMU.h"
#include "../../lib/PointCloudLiDAR/PointCloudLiDAR.h"
#include "../../lib/ColorSensor/ColorSensor.h"

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

typedef struct MapAddress MapAddress;

struct MapAddress {
	int x;
	int z;

	bool operator==(const MapAddress& other) const {
		return x == other.x && z == other.z;
	}
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
	vector<vector<string>> map_A = vector<vector<string>>(5, vector<string>(5, "-")); // �Ƃ肠����2x2�̃}�b�v�����
	// �������Ɠ������炢�厖����

	Map() {
		markTileAs({ 1, 1 }, TileState::START, 1); // �X�^�[�g�^�C�����}�[�N �p�x�͓K��
	}

	// �^�C���̒��S�ɓ��B�����珉�߂ɌĂ�
	void updatePostion(int dx_R, int dz_R);

	void updatePostion(const double& angle) {
		//cout << "upPos: " << currentTile_R.x << " " << currentTile_R.z << " " << angle << endl;
		if (abs(angle - 90) < 5) {
			updatePostion(1,0);
		}
		else if (abs(angle - 180) < 5) {
			updatePostion(0,-1);
		}
		else if (abs(angle - 270) < 5) {
			updatePostion(-1, 0);
		}
		else if ((angle >= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
			updatePostion(0, 1);
		}
		//cout << "upedPos: " << currentTile_R.x << " " << currentTile_R.z << " " << angle << endl;
	}

	void markTileAs(MapAddress add_R, TileState tilestate, const double& angle) {
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
		string tile = "0";
		
		// ���̏ꍇ�͖�������2������
		if (tilestate == TileState::HOLE) {
			if (map_A[add_L.z - 1][add_L.x - 1] == "0" || map_A[add_L.z - 1][add_L.x - 1] == "-") map_A[add_L.z - 1][add_L.x - 1] = "2";
			if (map_A[add_L.z - 1][add_L.x + 1] == "0" || map_A[add_L.z - 1][add_L.x + 1] == "-") map_A[add_L.z - 1][add_L.x + 1] = "2";
			if (map_A[add_L.z + 1][add_L.x - 1] == "0" || map_A[add_L.z + 1][add_L.x - 1] == "-") map_A[add_L.z + 1][add_L.x - 1] = "2";
			if (map_A[add_L.z + 1][add_L.x + 1] == "0" || map_A[add_L.z + 1][add_L.x + 1] == "-") map_A[add_L.z + 1][add_L.x + 1] = "2";

			// �ǂ̏�񂪓���}�X
			ifBarWrite(add_L.x, add_L.z - 1, "0");
			ifBarWrite(add_L.x, add_L.z + 1, "0");
			ifBarWrite(add_L.x - 1, add_L.z, "0");
			ifBarWrite(add_L.x + 1, add_L.z, "0");
			ifBarWrite(add_L.x, add_L.z, "0");
			return;
		}

		// ���������� or �����������Z�� or ����������p�x��X��
		if (( add_R.x % 2 == 0 && add_R.z % 2 == 0 ) ||
				( abs(add_R.x % 2) == 1 && add_R.z % 2 == 0 && (abs(angle - 180) < 5 || ((angle >= 0 && angle < 5) || (angle > 355 && angle <= 360)))) ||
				( add_R.x % 2 == 0 && abs(add_R.z % 2) == 1 && (abs(angle - 90) < 5 || abs(angle - 270) < 5))
			){
			if (!(add_R.x % 2 == 0 && add_R.z % 2 == 0)) tile = TileStateMap[tilestate];
			if (abs(angle - 90) < 5) {
				if (map_A[add_L.z - 1][add_L.x + 1] == "-") map_A[add_L.z - 1][add_L.x + 1] = tile; // ��3�s�v��񂩂�
				if (map_A[add_L.z    ][add_L.x + 1] == "-") map_A[add_L.z    ][add_L.x + 1] = "0";
				if (map_A[add_L.z + 1][add_L.x + 1] == "-") map_A[add_L.z + 1][add_L.x + 1] = tile;
				if (map_A[add_L.z - 1][add_L.x + 2] == "-") map_A[add_L.z - 1][add_L.x + 2] = "0";
				if (map_A[add_L.z    ][add_L.x + 2] == "-") map_A[add_L.z    ][add_L.x + 2] = "0";
				if (map_A[add_L.z + 1][add_L.x + 2] == "-") map_A[add_L.z + 1][add_L.x + 2] = "0";
			}
			else if (abs(angle - 180) < 5) {
				if (map_A[add_L.z - 1][add_L.x - 1] == "-") map_A[add_L.z - 1][add_L.x - 1] = tile;
				if (map_A[add_L.z - 1][add_L.x    ] == "-") map_A[add_L.z - 1][add_L.x    ] = "0";
				if (map_A[add_L.z - 1][add_L.x + 1] == "-") map_A[add_L.z - 1][add_L.x + 1] = tile;
				if (map_A[add_L.z - 2][add_L.x - 1] == "-") map_A[add_L.z - 2][add_L.x - 1] = "0";
				if (map_A[add_L.z - 2][add_L.x    ] == "-") map_A[add_L.z - 2][add_L.x    ] = "0";
				if (map_A[add_L.z - 2][add_L.x + 1] == "-") map_A[add_L.z - 2][add_L.x + 1] = "0";
			}
			else if (abs(angle - 270) < 5) {
				if (map_A[add_L.z - 1][add_L.x - 1] == "-") map_A[add_L.z - 1][add_L.x - 1] = tile;
				if (map_A[add_L.z    ][add_L.x - 1] == "-") map_A[add_L.z    ][add_L.x - 1] = "0";
				if (map_A[add_L.z + 1][add_L.x - 1] == "-") map_A[add_L.z + 1][add_L.x - 1] = tile;
				if (map_A[add_L.z - 1][add_L.x - 2] == "-") map_A[add_L.z - 1][add_L.x - 2] = "0";
				if (map_A[add_L.z    ][add_L.x - 2] == "-") map_A[add_L.z    ][add_L.x - 2] = "0";
				if (map_A[add_L.z + 1][add_L.x - 2] == "-") map_A[add_L.z + 1][add_L.x - 2] = "0";
			}
			else if ((angle >= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
				if (map_A[add_L.z + 1][add_L.x - 1] == "-") map_A[add_L.z + 1][add_L.x - 1] = tile;
				if (map_A[add_L.z + 1][add_L.x    ] == "-") map_A[add_L.z + 1][add_L.x    ] = "0";
				if (map_A[add_L.z + 1][add_L.x + 1] == "-") map_A[add_L.z + 1][add_L.x + 1] = tile;
				if (map_A[add_L.z + 2][add_L.x - 1] == "-") map_A[add_L.z + 2][add_L.x - 1] = "0";
				if (map_A[add_L.z + 2][add_L.x    ] == "-") map_A[add_L.z + 2][add_L.x    ] = "0";
				if (map_A[add_L.z + 2][add_L.x + 1] == "-") map_A[add_L.z + 2][add_L.x + 1] = "0";
			}
			return;
		}

		// �����������X�� or �����������Z��
		if ((add_R.x % 2 == 0 && abs(add_R.z % 2) == 1 && (abs(angle - 180) < 5 || (angle >= 0 && angle < 5) || (angle > 355 && angle <= 360))) ||
				(abs(add_R.x % 2) == 1 && add_R.z % 2 == 0 && (abs(angle - 90) < 5 || abs(angle - 270) < 5))) {
			ifBarWrite(add_L.x - 1, add_L.z - 1, "0");
			ifBarWrite(add_L.x + 1, add_L.z - 1, "0");
			ifBarWrite(add_L.x - 1, add_L.z + 1, "0");
			ifBarWrite(add_L.x + 1, add_L.z + 1, "0");
			ifBarWrite(add_L.x    , add_L.z - 1, "0");
			ifBarWrite(add_L.x    , add_L.z + 1, "0");
			ifBarWrite(add_L.x - 1, add_L.z    , "0");
			ifBarWrite(add_L.x + 1, add_L.z    , "0");
			ifBarWrite(add_L.x    , add_L.z    , "0");
			return;
		}

		// �����̊��
		tile = TileStateMap[tilestate];

		// �^�C���̏�񂪓���}�X
		if (map_A[add_L.z - 1][add_L.x - 1] == "0" || map_A[add_L.z - 1][add_L.x - 1] == "-") map_A[add_L.z - 1][add_L.x - 1] = tile;
		if (map_A[add_L.z - 1][add_L.x + 1] == "0" || map_A[add_L.z - 1][add_L.x + 1] == "-") map_A[add_L.z - 1][add_L.x + 1] = tile;
		if (map_A[add_L.z + 1][add_L.x - 1] == "0" || map_A[add_L.z + 1][add_L.x - 1] == "-") map_A[add_L.z + 1][add_L.x - 1] = tile;
		if (map_A[add_L.z + 1][add_L.x + 1] == "0" || map_A[add_L.z + 1][add_L.x + 1] == "-") map_A[add_L.z + 1][add_L.x + 1] = tile;

		// �ǂ̏�񂪓���}�X
		ifBarWrite(add_L.x, add_L.z -1, "0");
		ifBarWrite(add_L.x, add_L.z +1, "0");
		ifBarWrite(add_L.x -1, add_L.z, "0");
		ifBarWrite(add_L.x +1, add_L.z, "0");
		ifBarWrite(add_L.x, add_L.z, "0");
	}

	/* ���Ɏg���˂񂱂�{ */
	void markAroundTile(TileState front, TileState back, TileState left, TileState right, float angle = -1);

	TileState getTileState(MapAddress addr_R, int16_t relative_angle = 1);

	// ���̕ǂ܂ł͒��ׂȂ�
	void getAroundTileState(MapAddress addr_R, TileState& front, TileState& back, TileState& left, TileState& right, double angle = -1);

	// �Ƃ肠�����^�C�����ׂ��悤�ȋȐ��Ƃ��̕ςȕǂ͂Ȃ����̂Ƃ��čl����B
	void markAroundWall(WallSet NorthWall, WallSet SouthWall, WallSet WestWall, WallSet EastWall);

	// ������k�̕ǂ����Ă���
	void markNorthWall(MapAddress addr_R, WallSet wallset);
	void markSouthWall(MapAddress addr_R, WallSet wallset);
	void markWestWall(MapAddress addr_R, WallSet wallset);
	void markEastWall(MapAddress addr_R, WallSet wallset);

	// �ǂ̗L���A����`�����𒲂ׂ� ��̓I�Ȏ�ނ͔��ʂ��Ȃ� �g��Ȃ�
	void getAroundWallState(const MapAddress& addr_R, WallState& frontWall, WallState& backWall, WallState& leftWall, WallState& rightWall, double angle = -1);

	// �p�̏��� �g��Ȃ�
	void edge(MapAddress add_R, MapAddress add_L = { -1,-1 });

	void addNorth(const int i = 1); // �k�����Ƀ^�C����ǉ�
	void addSouth(const int i = 1); // ������Ƀ^�C����ǉ�
	void addWest(const int j = 1); // �������Ƀ^�C����ǉ�
	void addEast(const int j = 1); // �������Ƀ^�C����ǉ�

	//�@�S�̂����݂��邩
	bool existTile_R2(const MapAddress& addr_R) {
		MapAddress addr_L = convertRtoListPoint(addr_R);
		if (addr_L.x - 2 < 0) {
			cout << "existTile_R2" << endl;
			return false;
		}
		else if (addr_L.z - 2 < 0) {
			cout << "existTile_R2" << endl;
			return false;
		}
		else if (addr_L.x + 3 > map_A[0].size()) {
			cout << "existTile_R2" << endl;
			return false;
		}
		else if (addr_L.z + 3 > map_A.size()) {
			cout << "existTile_R2" << endl;
			return false;
		}
		return true;
	}

	// �}�b�v�S�̂�\������
	void printMap();

	// �Ō��"-"��"0"�ɕύX���ď����ł��|�C���g���҂����Ƒ��~��
	void replaceLineTo0();

	MapAddress startTile_A = { 1, 1 }, startTile_R = { 1, 1 };
	MapAddress currentTile_A = { 1, 1 }, currentTile_R = { 1, 1 };
	unordered_set<TileState> ColoredTiles = { TileState::CHECKPOINT, TileState::AREA1to2, TileState::AREA2to3, TileState::AREA3to4, TileState::AREA1to4, TileState::SWAMP };

private: // ************************************************************************************
	/* �Ȃ񂾃R�� */
	int convertTiletoList(int tile) {
		return tile * 4;
	}

	/* ��΍��W�𑊑΍��W�ɕϊ� */
	MapAddress convertAtoR(const MapAddress& addr_A) {
		return { addr_A.x - startTile_A.x + 1, addr_A.z - startTile_A.z + 1 };
	}

	/* ���΍��W���΍��W�ɕϊ� */
	MapAddress convertRtoA(const MapAddress& addr_R) {
		return { addr_R.x + startTile_A.x -1, addr_R.z + startTile_A.z -1 };
	}

	/* �^�C�����W���烊�X�g�̃^�C�����S�̍��W�ɕϊ� */
	MapAddress convertAtoListPoint(const MapAddress& addr_A) {
		return { addr_A.x * 2, addr_A.z * 2 };
	}

	/* ���X�g�̃^�C�����S�̍��W����^�C�����W�ɕϊ� */
	MapAddress convertListPointtoA(const MapAddress& addr_list) {
		return { (addr_list.x) / 2, (addr_list.z) / 2 };
	}

	/* ���΍��W���烊�X�g�̃^�C�����S�̍��W�ɕϊ� */
	MapAddress convertRtoListPoint(const MapAddress& addr_R) {
		return convertAtoListPoint(convertRtoA(addr_R));
	}

	/* ���X�g�̃^�C�����S�̍��W���瑊�΍��W�ɕϊ� */
	MapAddress convertListPointtoR(const MapAddress& addr_list) {
		return convertAtoR(convertListPointtoA(addr_list));
	}

	// ���S�����݂��邩
	bool existTile_R(const MapAddress& addr_R) {
		MapAddress addr_L = convertRtoListPoint(addr_R);
		if (addr_L.x < 0) {
			return false;
		}
		else if (addr_L.z < 0) {
			return false;
		}
		else if (addr_L.x >= map_A[0].size()) {
			return false;
		}
		else if (addr_L.z >= map_A.size()) {
			return false;
		}
		return true;
	}

	void rotate90Degrees(vector<vector<string>>& arr);

	void paintTile(vector<vector<string>>& tile, const WallSet& wallset);
	void drawTile(vector<vector<string>>& tile, MapAddress add_L);
	bool isBackClear(vector<vector<string>>& tile);

	void write_map_A(int x, int z, string part) {
		if (z < 0 || x < 0 || z >= map_A.size() || x >= map_A[0].size()) {
			while (robot->step(timeStep) != -1) {
				cout << "can not write map_A !! " << endl;
				cout << "x: " << x << " z: " << z << " x size: " << map_A[0].size() << " z size: " << map_A.size() << endl;
				robot->step(timeStep * 100);
			}
		}
		map_A[z][x] = part;
	}

	void ifBarWrite(int x, int z, string part) {
		if (map_A[z][x] == "-") {
			map_A[z][x] = part;
		}
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

	std::map<string,TileState> TileStateMap2 = {
		{"0",TileState::OTHER},
		{"1",TileState::WALL},
		{"2",TileState::HOLE},
		{"3",TileState::SWAMP},
		{"4",TileState::CHECKPOINT},
		{"5",TileState::START},
		{"6",TileState::AREA1to2},
		{"7",TileState::AREA2to3},
		{"8",TileState::AREA3to4},
		{"9",TileState::AREA1to4},
		{"-",TileState::UNKNOWN},
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
	MapAddress left_top_R = { 1, 1 }, right_bottom_R = { 1, 1 };
};