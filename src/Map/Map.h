#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <math.h>
#include <vector>
#include <algorithm>
//#include <deque>
#include <map>

#include "../../lib/GlobalPositioningSystem/GlobalPositioningSystem.h"

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
	UNKNOWN,
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
		markTileAs({ 0, 0 }, getTileName(TileState::START));
	}

	void markTileAs(MapAddress add_R, string tile) {
		MapAddress add_L = convertRtoListPoint(add_R);
		map_A[add_L.z - 1][add_L.x - 1] = tile;
		map_A[add_L.z - 1][add_L.x + 1] = tile;
		map_A[add_L.z + 1][add_L.x - 1] = tile;
		map_A[add_L.z + 1][add_L.x + 1] = tile;

		map_A[add_L.z - 1][add_L.x] = "1";
		map_A[add_L.z + 1][add_L.x] = "1";
		map_A[add_L.z][add_L.x - 1] = "1";
		map_A[add_L.z][add_L.x + 1] = "1";
		map_A[add_L.z][add_L.x] = "1";
	}

	void addNorth(int i = 1) { // ��ɒǉ�
		// map_A�̉���4i�s�ǉ�
		cout << "map_A.size() = " << map_A.size() << endl;
		map_A.resize(map_A.size() + 4 * i, vector<string>(map_A[0].size(), "-"));
		// map_A�̗v�f������4i�����炷
		rotate(map_A.rbegin(), map_A.rbegin() + 4 * i, map_A.rend());
		startTile_A.z++;
		currentTile_A.z++;
	}

	void addSouth(int i) { // ���ɒǉ�
		// map_A�̉���1�s�ǉ�
		cout << "map_A.size() = " << map_A.size() << endl;
		map_A.resize(map_A.size() + 4 * i, vector<string>(map_A[0].size(), "-"));
	}

	void addWest(int j = 1) { // ���ɒǉ�
		// �����̍s�ɐV�������ǉ�
		for (int i = 0; i < map_A.size(); ++i) {
			map_A[i].resize(map_A[i].size() + 4 * j, "-");
		}
		// �S�̗̂v�f������1���炷
		for (int i = 0; i < map_A.size(); ++i) {
			rotate(map_A[i].rbegin(), map_A[i].rbegin() + 4 * j, map_A[i].rend());
		}
		startTile_A.x++;
		currentTile_A.x++;
	}

	void addEast(int j = 1) { // �E�ɒǉ�
		// �����̍s�ɐV�������ǉ�
		for (int i = 0; i < map_A.size(); ++i) {
			map_A[i].resize(map_A[i].size() + 4 * j, "-");
		}
	}

	string getTileName(TileState state) {
		return TileStateMap[state];
	}

	string getVictimName(VictimState state) {
		return VictimStateMap[state];
	}

	void printMap() {
		for (int i = 0; i < map_A.size(); ++i) {
			for (int j = 0; j < map_A[i].size(); ++j) {
				cout << map_A[i][j] << " ";
			}
			cout << endl;
		}
	}
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