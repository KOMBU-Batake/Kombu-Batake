#include "Map2.h"

void Map2::addNorth(const int i) { // ��ɒǉ�
	// map_A�̉���4i�s�ǉ�
	//cout << "map_A.size() = " << map_A.size() << endl;
	map_A.resize(map_A.size() + 4 * i, vector<string>(map_A[0].size(), "-"));
	// map_A�̗v�f������4i�����炷
	rotate(map_A.rbegin(), map_A.rbegin() + 4 * i, map_A.rend());
	startTile_A.z++;
	currentTile_A.z++;
	left_top_R.z--;
}

void Map2::addSouth(const int i) { // ���ɒǉ�
	// map_A�̉���1�s�ǉ�
	//cout << "map_A.size() = " << map_A.size() << endl;
	map_A.resize(map_A.size() + 4 * i, vector<string>(map_A[0].size(), "-"));
	right_bottom_R.z += 1;
}

void Map2::addWest(const int j) { // ���ɒǉ�
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
	left_top_R.x--;
}

void Map2::addEast(const int j) { // �E�ɒǉ�
	// �����̍s�ɐV�������ǉ�
	for (int i = 0; i < map_A.size(); ++i) {
		map_A[i].resize(map_A[i].size() + 4 * j, "-");
	}
	right_bottom_R.x++;
}