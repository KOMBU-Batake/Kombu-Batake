#include "Map.h"

void Map::addNorth(const int i) { // ��ɒǉ�
	// map_A�̉���4i�s�ǉ�
	//cout << "map_A.size() = " << map_A.size() << endl;
	map_A.resize(map_A.size() + 4 * i, vector<string>(map_A[0].size(), "-"));
	// map_A�̗v�f������4i�����炷
	rotate(map_A.rbegin(), map_A.rbegin() + 4 * i, map_A.rend());
	startTile_A.z++;
	currentTile_A.z++;
	left_top_R.z--;
}

void Map::addSouth(const int i) { // ���ɒǉ�
	// map_A�̉���1�s�ǉ�
	//cout << "map_A.size() = " << map_A.size() << endl;
	map_A.resize(map_A.size() + 4 * i, vector<string>(map_A[0].size(), "-"));
	right_bottom_R.z++;
}

void Map::addWest(const int j) { // ���ɒǉ�
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

void Map::addEast(const int j) { // �E�ɒǉ�
	// �����̍s�ɐV�������ǉ�
	for (int i = 0; i < map_A.size(); ++i) {
		map_A[i].resize(map_A[i].size() + 4 * j, "-");
	}
	right_bottom_R.x++;
}

void Map::printMap() {
	for (int i = 0; i < map_A.size(); ++i) {
		for (int j = 0; j < map_A[i].size(); ++j) {
			cout << map_A[i][j] << " ";
		}
		cout << endl;
	}
}

void Map::markAroundTile(TileState front, TileState back, TileState left, TileState right, float angle) { // ���ݒn�͍X�V����Ă���O��
	MapAddress tmpAddr = currentTile_R;
	if (angle == -1) angle = (float)gyro.getGyro();
	if (abs(angle - 90) < 5) {
		markTileAs({ tmpAddr.x + 1, tmpAddr.z }, front);
		markTileAs({ tmpAddr.x - 1, tmpAddr.z }, back);
		markTileAs({ tmpAddr.x, tmpAddr.z - 1 }, left);
		markTileAs({ tmpAddr.x, tmpAddr.z + 1 }, right);
	}
	else if (abs(angle - 180) < 5) {
		markTileAs({ tmpAddr.x, tmpAddr.z - 1 }, front);
		markTileAs({ tmpAddr.x, tmpAddr.z + 1 }, back);
		markTileAs({ tmpAddr.x - 1, tmpAddr.z }, left);
		markTileAs({ tmpAddr.x + 1, tmpAddr.z }, right);
	}
	else if (abs(angle - 270) < 5) {
		markTileAs({ tmpAddr.x - 1, tmpAddr.z }, front);
		markTileAs({ tmpAddr.x + 1, tmpAddr.z }, back);
		markTileAs({ tmpAddr.x, tmpAddr.z + 1 }, left);
		markTileAs({ tmpAddr.x, tmpAddr.z - 1 }, right);
	}
	else if ((angle <= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
		markTileAs({ tmpAddr.x, tmpAddr.z + 1 }, front);
		markTileAs({ tmpAddr.x, tmpAddr.z - 1 }, back);
		markTileAs({ tmpAddr.x + 1, tmpAddr.z }, left);
		markTileAs({ tmpAddr.x - 1, tmpAddr.z }, right);
	}
	else cout << "unreliable angle in makeAround4" << endl;
}

void Map::replaceLineTo0() {
	for (auto& row : map_A) {
		std::replace(row.begin(), row.end(), "-", "0");
	}
}

TileState Map::getTileState(MapAddress addr_R) {
	MapAddress addr_L = convertRtoListPoint(addr_R);
	return static_cast<TileState>(stoi(map_A[addr_L.z + 1][addr_L.x + 1]));
}