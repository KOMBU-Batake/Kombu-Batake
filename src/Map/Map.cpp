#include "Map.h"

void Map::updatePostion(int dx_R, int dz_R) {
	currentTile_R.x += dx_R;
	currentTile_R.z += dz_R;
	// マップ範囲外の場合は拡張
	if (currentTile_R.x < left_top_R.x) {
		addWest(left_top_R.x - currentTile_R.x);
	}
	else if (currentTile_R.x > right_bottom_R.x) {
		addEast(currentTile_R.x - right_bottom_R.x);
	}
	if (currentTile_R.z < left_top_R.z) {
		addNorth(left_top_R.z - currentTile_R.z);
	}
	else if (currentTile_R.z > right_bottom_R.z) {
		addSouth(currentTile_R.z - right_bottom_R.z);
	}
}

void Map::addNorth(const int i) { // 上に追加
	// map_Aの下に4i行追加
	//cout << "map_A.size() = " << map_A.size() << endl;
	map_A.resize(map_A.size() + 4 * i, vector<string>(map_A[0].size(), "-"));
	// map_Aの要素を下に4iずつずらす
	rotate(map_A.rbegin(), map_A.rbegin() + 4 * i, map_A.rend());
	startTile_A.z++;
	currentTile_A.z++;
	left_top_R.z--;
}

void Map::addSouth(const int i) { // 下に追加
	// map_Aの下に1行追加
	//cout << "map_A.size() = " << map_A.size() << endl;
	map_A.resize(map_A.size() + 4 * i, vector<string>(map_A[0].size(), "-"));
	right_bottom_R.z++;
}

void Map::addWest(const int j) { // 左に追加
	// 既存の行に新しい列を追加
	for (int i = 0; i < map_A.size(); ++i) {
		map_A[i].resize(map_A[i].size() + 4 * j, "-");
	}
	// 全体の要素を左に1つずらす
	for (int i = 0; i < map_A.size(); ++i) {
		rotate(map_A[i].rbegin(), map_A[i].rbegin() + 4 * j, map_A[i].rend());
	}
	startTile_A.x++;
	currentTile_A.x++;
	left_top_R.x--;
}

void Map::addEast(const int j) { // 右に追加
	// 既存の行に新しい列を追加
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

void Map::markAroundTile(TileState front, TileState back, TileState left, TileState right, float angle) { // 現在地は更新されている前提
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

void Map::markAroundWall(WallState NorthWall, WallState SouthWall, WallState WestWall, WallState EastWall) {
	markNorthWall(currentTile_R, NorthWall);
	markSouthWall(currentTile_R, SouthWall);
	markWestWall(currentTile_R, WestWall);
	markEastWall(currentTile_R, EastWall);
	edge(currentTile_R);
}

void Map::markNorthWall(MapAddress addr_R, WallState wall) {
	MapAddress add_L = convertRtoListPoint(addr_R);
	vector<string> tmp = WallStateMap[wall];
	map_A[add_L.z - 2][add_L.x - 1] = tmp[0];
	map_A[add_L.z - 2][add_L.x] = tmp[1];
	map_A[add_L.z - 2][add_L.x + 1] = tmp[2];
}

void Map::markSouthWall(MapAddress addr_R, WallState wall) {
	MapAddress add_L = convertRtoListPoint(addr_R);
	vector<string> tmp = WallStateMap[wall];
	map_A[add_L.z + 2][add_L.x + 1] = tmp[0];
	map_A[add_L.z + 2][add_L.x] = tmp[1];
	map_A[add_L.z + 2][add_L.x - 1] = tmp[2];
}

void Map::markWestWall(MapAddress addr_R, WallState wall) {
	MapAddress add_L = convertRtoListPoint(addr_R);
	vector<string> tmp = WallStateMap[wall];
	map_A[add_L.z + 1][add_L.x - 2] = tmp[0];
	map_A[add_L.z][add_L.x - 2] = tmp[1];
	map_A[add_L.z - 1][add_L.x - 2] = tmp[2];
}

void Map::markEastWall(MapAddress addr_R, WallState wall) {
	MapAddress add_L = convertRtoListPoint(addr_R);
	vector<string> tmp = WallStateMap[wall];
	map_A[add_L.z - 1][add_L.x + 2] = tmp[0];
	map_A[add_L.z][add_L.x + 2] = tmp[1];
	map_A[add_L.z + 1][add_L.x + 2] = tmp[2];
}

void Map::edge(MapAddress add_R, MapAddress add_L) {
	if (add_L.x == -1) add_L = convertRtoListPoint(add_R);
	// 左上
	if (WallAndVictim.find(map_A[add_L.z - 1][add_L.x - 2]) != WallAndVictim.end() ||
		WallAndVictim.find(map_A[add_L.z - 2][add_L.x - 1]) != WallAndVictim.end()) {
		map_A[add_L.z - 2][add_L.x - 2] = "1";
	} else if (map_A[add_L.z - 2][add_L.x - 2] == "-") map_A[add_L.z - 2][add_L.x - 2] = "0";
	// 右上
	if (WallAndVictim.find(map_A[add_L.z - 1][add_L.x + 2]) != WallAndVictim.end() ||
		WallAndVictim.find(map_A[add_L.z - 2][add_L.x + 1]) != WallAndVictim.end()) {
		map_A[add_L.z - 2][add_L.x + 2] = "1";
	} else if (map_A[add_L.z - 2][add_L.x + 2] == "-") map_A[add_L.z - 2][add_L.x + 2] = "0";
	// 左下
	if (WallAndVictim.find(map_A[add_L.z + 1][add_L.x - 2]) != WallAndVictim.end() ||
		WallAndVictim.find(map_A[add_L.z + 2][add_L.x - 1]) != WallAndVictim.end()) {
		map_A[add_L.z + 2][add_L.x - 2] = "1";
	} else if (map_A[add_L.z + 2][add_L.x - 2] == "-") map_A[add_L.z + 2][add_L.x - 2] = "0";
	// 右下
	if (WallAndVictim.find(map_A[add_L.z + 1][add_L.x + 2]) != WallAndVictim.end() ||
		WallAndVictim.find(map_A[add_L.z + 2][add_L.x + 1]) != WallAndVictim.end()) {
		map_A[add_L.z + 2][add_L.x + 2] = "1";
	} else if (map_A[add_L.z + 2][add_L.x + 2] == "-") map_A[add_L.z + 2][add_L.x + 2] = "0";
}

void Map::getAroundWallState(const MapAddress& addr_R, WallState& frontWall, WallState& backWall, WallState& leftWall, WallState& rightWall, double angle) {
	if (angle == -1) {
		angle = gyro.getGyro();
	}
	MapAddress add_L = convertRtoListPoint(addr_R);
	frontWall = WallState::noWALL;
	backWall = WallState::noWALL;
	leftWall = WallState::noWALL;
	rightWall = WallState::noWALL;
	if (abs(angle - 90) < 5) {
		if (WallAndVictim.find(map_A[add_L.z][add_L.x + 2]) != WallAndVictim.end()) frontWall = WallState::WALL;
		if (WallAndVictim.find(map_A[add_L.z][add_L.x - 2]) != WallAndVictim.end()) backWall = WallState::WALL;
		if (WallAndVictim.find(map_A[add_L.z - 2][add_L.x]) != WallAndVictim.end()) leftWall = WallState::WALL;
		if (WallAndVictim.find(map_A[add_L.z + 2][add_L.x]) != WallAndVictim.end()) rightWall = WallState::WALL;
	}
	else if (abs(angle - 180) < 5) {
		if (WallAndVictim.find(map_A[add_L.z - 2][add_L.x]) != WallAndVictim.end()) frontWall = WallState::WALL;
		if (WallAndVictim.find(map_A[add_L.z + 2][add_L.x]) != WallAndVictim.end()) backWall = WallState::WALL;
		if (WallAndVictim.find(map_A[add_L.z][add_L.x - 2]) != WallAndVictim.end()) leftWall = WallState::WALL;
		if (WallAndVictim.find(map_A[add_L.z][add_L.x + 2]) != WallAndVictim.end()) rightWall = WallState::WALL;
	}
	else if (abs(angle - 270) < 5) {
		if (WallAndVictim.find(map_A[add_L.z][add_L.x - 2]) != WallAndVictim.end()) frontWall = WallState::WALL;
		if (WallAndVictim.find(map_A[add_L.z][add_L.x + 2]) != WallAndVictim.end()) backWall = WallState::WALL;
		if (WallAndVictim.find(map_A[add_L.z + 2][add_L.x]) != WallAndVictim.end()) leftWall = WallState::WALL;
		if (WallAndVictim.find(map_A[add_L.z - 2][add_L.x]) != WallAndVictim.end()) rightWall = WallState::WALL;
	}
	else if ((angle <= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
		if (WallAndVictim.find(map_A[add_L.z + 2][add_L.x]) != WallAndVictim.end()) frontWall = WallState::WALL;
		if (WallAndVictim.find(map_A[add_L.z - 2][add_L.x]) != WallAndVictim.end()) backWall = WallState::WALL;
		if (WallAndVictim.find(map_A[add_L.z][add_L.x + 2]) != WallAndVictim.end()) leftWall = WallState::WALL;
		if (WallAndVictim.find(map_A[add_L.z][add_L.x - 2]) != WallAndVictim.end()) rightWall = WallState::WALL;
	}
}

void Map::replaceLineTo0() {
	for (auto& row : map_A) {
		std::replace(row.begin(), row.end(), "-", "0");
	}
}

TileState Map::getTileState(MapAddress addr_R) {
	if (!existTile_R(addr_R)) {
		return TileState::UNKNOWN;
	}
	MapAddress addr_L = convertRtoListPoint(addr_R);
	return static_cast<TileState>(stoi(map_A[addr_L.z + 1][addr_L.x + 1]));
}

void Map::getAroundTileState(MapAddress addr_R, TileState& front, TileState& back, TileState& left, TileState& right, double angle) {
	if (angle == -1) {
		angle = gyro.getGyro();
	}
	if (abs(angle - 90) < 5) {
		front = getTileState({ addr_R.x + 1, addr_R.z });
		back = getTileState({ addr_R.x - 1, addr_R.z });
		left = getTileState({ addr_R.x, addr_R.z - 1 });
		right = getTileState({ addr_R.x, addr_R.z + 1 });
	}
	else if (abs(angle - 180) < 5) {
		front = getTileState({ addr_R.x, addr_R.z - 1 });
		back = getTileState({ addr_R.x, addr_R.z + 1 });
		left = getTileState({ addr_R.x - 1, addr_R.z });
		right = getTileState({ addr_R.x + 1, addr_R.z });
	}
	else if (abs(angle - 270) < 5) {
		front = getTileState({ addr_R.x - 1, addr_R.z });
		back = getTileState({ addr_R.x + 1, addr_R.z });
		left = getTileState({ addr_R.x, addr_R.z + 1 });
		right = getTileState({ addr_R.x, addr_R.z - 1 });
	}
	else if ((angle <= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
		front = getTileState({ addr_R.x, addr_R.z + 1 });
		back = getTileState({ addr_R.x, addr_R.z - 1 });
		left = getTileState({ addr_R.x + 1, addr_R.z });
		right = getTileState({ addr_R.x - 1, addr_R.z });
	}
	else cout << "unreliable angle in getAroundTileState" << endl;
}