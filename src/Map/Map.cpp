#include "Map.h"

void Map::updatePostion(int dx_R, int dz_R) {
	currentTile_R.x += dx_R;
	currentTile_R.z += dz_R;
	currentTile_A.x += dx_R;
	currentTile_A.z += dz_R;
	mapperS.updatePostion(dx_R, dz_R);
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
	startTile_A.z += 2;
	currentTile_A.z += 2;
	left_top_R.z -= 2;
	mapperS.addNorth(i);
}

void Map::addSouth(const int i) { // 下に追加
	// map_Aの下に1行追加
	//cout << "map_A.size() = " << map_A.size() << endl;
	map_A.resize(map_A.size() + 4 * i, vector<string>(map_A[0].size(), "-"));
	right_bottom_R.z += 2;
	mapperS.addSouth(i);
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
	startTile_A.x += 2;
	currentTile_A.x += 2;
	left_top_R.x -= 2;
	mapperS.addWest(j);
}

void Map::addEast(const int j) { // 右に追加
	// 既存の行に新しい列を追加
	for (int i = 0; i < map_A.size(); ++i) {
		map_A[i].resize(map_A[i].size() + 4 * j, "-");
	}
	right_bottom_R.x += 2;
	mapperS.addEast(j);
}

void Map::printMap() {
	MapAddress add_L = convertRtoListPoint(currentTile_R);
	cout << "currentTile_R = (" << currentTile_R.x << ", " << currentTile_R.z << ")" << endl;
	cout << "currentTile_A = (" << currentTile_A.x << ", " << currentTile_A.z << ")" << endl;
	//cout << "add_L = (" << add_L.x << ", " << add_L.z << ")" << endl;
	for (int i = 0; i < map_A.size(); ++i) {
		for (int j = 0; j < map_A[i].size(); ++j) {
			if (add_L.z == i && add_L.x == j) {
				if (map_A[i][j] == "1") { cout << "G "; } else cout << "R ";
			} else cout << map_A[i][j] << " ";
		}
		cout << endl;
	}
}

void Map::markAroundTile(TileState front, TileState back, TileState left, TileState right, float angle) { // 現在地は更新されている前提
	MapAddress tmpAddr = currentTile_R;
	if (angle == -1) angle = (float)gyro.getGyro();
	if (abs(angle - 90) < 5) {
		markTileAs({ tmpAddr.x + 2, tmpAddr.z }, front, angle);
		markTileAs({ tmpAddr.x - 2, tmpAddr.z }, back, angle);
		markTileAs({ tmpAddr.x, tmpAddr.z - 2 }, left, angle);
		markTileAs({ tmpAddr.x, tmpAddr.z + 2 }, right, angle);
	}
	else if (abs(angle - 180) < 5) {
		markTileAs({ tmpAddr.x, tmpAddr.z - 2 }, front, angle);
		markTileAs({ tmpAddr.x, tmpAddr.z + 2 }, back, angle);
		markTileAs({ tmpAddr.x - 2, tmpAddr.z }, left, angle);
		markTileAs({ tmpAddr.x + 2, tmpAddr.z }, right, angle);
	}
	else if (abs(angle - 270) < 5) {
		markTileAs({ tmpAddr.x - 2, tmpAddr.z }, front, angle);
		markTileAs({ tmpAddr.x + 2, tmpAddr.z }, back, angle);
		markTileAs({ tmpAddr.x, tmpAddr.z + 2 }, left, angle);
		markTileAs({ tmpAddr.x, tmpAddr.z - 2 }, right, angle);
	}
	else if ((angle >= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
		markTileAs({ tmpAddr.x, tmpAddr.z + 2 }, front, angle);
		markTileAs({ tmpAddr.x, tmpAddr.z - 2 }, back, angle);
		markTileAs({ tmpAddr.x + 2, tmpAddr.z }, left, angle);
		markTileAs({ tmpAddr.x - 2, tmpAddr.z }, right, angle);
	}
	else cout << "unreliable angle in makeAround4" << endl;
}

void Map::markAroundWall(WallSet NorthWall, WallSet SouthWall, WallSet WestWall, WallSet EastWall) {
	markNorthWall(currentTile_R, NorthWall);
	markSouthWall(currentTile_R, SouthWall);
	markWestWall(currentTile_R, WestWall);
	markEastWall(currentTile_R, EastWall);
}

void Map::markNorthWall(MapAddress addr_R, WallSet wallset) {
	MapAddress add_L = convertRtoListPoint(addr_R);
	if (wallset.left == WallType::type10 && wallset.center == WallType::center_n && wallset.right == WallType::type10) {
		map_A[add_L.z - 2][add_L.x - 2] = "1";
		map_A[add_L.z - 2][add_L.x - 1] = "1";
		map_A[add_L.z - 2][add_L.x] = "1";
		map_A[add_L.z - 2][add_L.x + 1] = "1";
		map_A[add_L.z - 2][add_L.x + 2] = "1";
	}
	else if (wallset == WallSet{ WallType::typeNo, WallType::center_n, WallType::typeNo }) {
		if (!existTile_R2({ addr_R.x,addr_R.z - 2 })) addNorth(1);
		add_L = convertRtoListPoint(addr_R);
		map_A[add_L.z - 2][add_L.x - 1] = "0";
		map_A[add_L.z - 2][add_L.x    ] = "0";
		map_A[add_L.z - 2][add_L.x + 1] = "0";
		map_A[add_L.z - 3][add_L.x    ] = "0";
		map_A[add_L.z - 4][add_L.x    ] = "0";
		map_A[add_L.z - 5][add_L.x    ] = "0";
		map_A[add_L.z - 4][add_L.x + 1] = "0";
		map_A[add_L.z - 4][add_L.x - 1] = "0";
	}
	else if ((wallset.left == WallType::type16 && wallset.right == WallType::type17) ||
		(wallset.left == WallType::type16 && wallset.right == WallType::type10) ||
		(wallset.left == WallType::type10 && wallset.right == WallType::type17)) {
		return;
	}
	else
	{
		if (!existTile_R({ addr_R.x,addr_R.z - 2 })) addNorth(1);
		vector<vector<string>> wall = {
				{"-", "-", "-", "-", "-"},
				{"-", "-", "-", "-", "-"},
				{"-", "-", "-", "-", "-"},
				{"-", "-", "-", "-", "-"},
				{"-", "-", "-", "-", "-"},
		};
		paintTile(wall, wallset);
		bool isOnlyFront = isBackClear(wall);
		// rotate90Degrees();
		add_L = convertRtoListPoint(addr_R);
		if (isOnlyFront) {
			add_L.z -= 4;
			for (int i = 0; i <= 2; ++i) {
				for (int j = -2; j <= 2; ++j) {
					if (map_A[add_L.z + i][add_L.x + j] == "-" || map_A[add_L.z + i][add_L.x + j] == "0" || map_A[add_L.z + i][add_L.x + j] == "1") {
						if (wall[i + 2][j + 2] != "-") write_map_A(add_L.x + j, add_L.z + i, wall[i + 2][j + 2]);
						
					}
				}
			}
		}
		else {
			if (!existTile_R2({ addr_R.x,addr_R.z - 2 })) addNorth(1);
			add_L = convertRtoListPoint(addr_R);
			drawTile(wall, { add_L.x,add_L.z - 4 });
		}
	}
}

void Map::markSouthWall(MapAddress addr_R, WallSet wallset) {
	MapAddress add_L = convertRtoListPoint(addr_R);
	if (wallset.left == WallType::type10 && wallset.center == WallType::center_n && wallset.right == WallType::type10) {
		map_A[add_L.z + 2][add_L.x - 2] = "1";
		map_A[add_L.z + 2][add_L.x - 1] = "1";
		map_A[add_L.z + 2][add_L.x] = "1";
		map_A[add_L.z + 2][add_L.x + 1] = "1";
		map_A[add_L.z + 2][add_L.x + 2] = "1";
	}
	else if (wallset == WallSet{ WallType::typeNo, WallType::center_n, WallType::typeNo }) {
		if (!existTile_R2({ addr_R.x,addr_R.z + 2 })) addSouth(1);
		add_L = convertRtoListPoint(addr_R);
		map_A[add_L.z + 2][add_L.x - 1] = "0";
		map_A[add_L.z + 2][add_L.x    ] = "0";
		map_A[add_L.z + 2][add_L.x + 1] = "0";
		map_A[add_L.z + 3][add_L.x    ] = "0";
		map_A[add_L.z + 4][add_L.x    ] = "0";
		map_A[add_L.z + 5][add_L.x    ] = "0";
		map_A[add_L.z + 4][add_L.x + 1] = "0";
		map_A[add_L.z + 4][add_L.x - 1] = "0";
	}
	else if ((wallset.left == WallType::type16 && wallset.right == WallType::type17) ||
		(wallset.left == WallType::type16 && wallset.right == WallType::type10) ||
		(wallset.left == WallType::type10 && wallset.right == WallType::type17)) {
		return;
	}
	else 
	{
		if (!existTile_R({ addr_R.x,addr_R.z + 2 })) addSouth(1);
		vector<vector<string>> wall = {
			{"-", "-", "-", "-", "-"},
			{"-", "-", "-", "-", "-"},
			{"-", "-", "-", "-", "-"},
			{"-", "-", "-", "-", "-"},
			{"-", "-", "-", "-", "-"},
		};
		paintTile(wall, wallset);
		bool isOnlyFront = isBackClear(wall);
		rotate90Degrees(wall);
		rotate90Degrees(wall);
		add_L = convertRtoListPoint(addr_R);
		if (isOnlyFront) {
			add_L.z += 4;
			for (int8_t i = -2; i <= 0; i++) {
				for (int8_t j = -2; j <= 2; j++){
					if (map_A[add_L.z + i][add_L.x + j] == "-" || map_A[add_L.z + i][add_L.x + j] == "0" || map_A[add_L.z + i][add_L.x + j] == "1") {
						if (wall[i + 2][j + 2] != "-") write_map_A(add_L.x + j, add_L.z + i, wall[i + 2][j + 2]);
					}
				}
			}
		}
		else {
			if (!existTile_R2({ addr_R.x,addr_R.z + 2 })) addSouth(1);
			add_L = convertRtoListPoint(addr_R);
			drawTile(wall, { add_L.x,add_L.z + 4 });
		}
	}
}

void Map::markWestWall(MapAddress addr_R, WallSet wallset) {
	MapAddress add_L = convertRtoListPoint(addr_R);
	if (wallset.left == WallType::type10 && wallset.center == WallType::center_n && wallset.right == WallType::type10) {
		map_A[add_L.z - 2][add_L.x - 2] = "1";
		map_A[add_L.z - 1][add_L.x - 2] = "1";
		map_A[add_L.z    ][add_L.x - 2] = "1";
		map_A[add_L.z + 1][add_L.x - 2] = "1";
		map_A[add_L.z + 2][add_L.x - 2] = "1";
	}
	else if (wallset == WallSet{ WallType::typeNo, WallType::center_n, WallType::typeNo }) {
		if (!existTile_R2({ addr_R.x - 2,addr_R.z })) addWest(1);
		add_L = convertRtoListPoint(addr_R);
		map_A[add_L.z - 1][add_L.x - 2] = "0";
		map_A[add_L.z    ][add_L.x - 2] = "0";
		map_A[add_L.z + 1][add_L.x - 2] = "0";
		map_A[add_L.z		 ][add_L.x - 3] = "0";
		map_A[add_L.z		 ][add_L.x - 4] = "0";
		map_A[add_L.z		 ][add_L.x - 5] = "0";
		map_A[add_L.z - 1][add_L.x - 4] = "0";
		map_A[add_L.z + 1][add_L.x - 4] = "0";
	}
	else if ((wallset.left == WallType::type16 && wallset.right == WallType::type17) ||
		(wallset.left == WallType::type16 && wallset.right == WallType::type10) ||
		(wallset.left == WallType::type10 && wallset.right == WallType::type17)) {
		return;
	}
	else 
	{
		if (!existTile_R({ addr_R.x - 2,addr_R.z })) addWest(1);
		vector<vector<string>> wall = {
			{"-", "-", "-", "-", "-"},
			{"-", "-", "-", "-", "-"},
			{"-", "-", "-", "-", "-"},
			{"-", "-", "-", "-", "-"},
			{"-", "-", "-", "-", "-"},
		};
		paintTile(wall, wallset);
		bool isOnlyFront = isBackClear(wall);
		rotate90Degrees(wall);
		rotate90Degrees(wall);
		rotate90Degrees(wall);
		add_L = convertRtoListPoint(addr_R);
		if (isOnlyFront) {
			add_L.x -= 4;
			for (int8_t i = -2; i <= 2; i++) {
				for (int8_t j = 0; j <= 2; j++) {
					if (map_A[add_L.z + i][add_L.x + j] == "-" || map_A[add_L.z + i][add_L.x + j] == "0" || map_A[add_L.z + i][add_L.x + j] == "1") {
						if (wall[i + 2][j + 2] != "-") write_map_A(add_L.x + j, add_L.z + i, wall[i + 2][j + 2]);
					}
				}
			}
		}
		else {
			if (!existTile_R2({ addr_R.x - 2,addr_R.z })) addWest(1);
			add_L = convertRtoListPoint(addr_R);
			drawTile(wall, { add_L.x - 4,add_L.z });
		}
	}
}

void Map::markEastWall(MapAddress addr_R, WallSet wallset) {
	MapAddress add_L = convertRtoListPoint(addr_R);
if (wallset.left == WallType::type10 && wallset.center == WallType::center_n && wallset.right == WallType::type10) {
		map_A[add_L.z - 2][add_L.x + 2] = "1";
		map_A[add_L.z - 1][add_L.x + 2] = "1";
		map_A[add_L.z    ][add_L.x + 2] = "1";
		map_A[add_L.z + 1][add_L.x + 2] = "1";
		map_A[add_L.z + 2][add_L.x + 2] = "1";
	}
	else if (wallset == WallSet{ WallType::typeNo, WallType::center_n, WallType::typeNo }) {
		if (!existTile_R2({ addr_R.x + 2,addr_R.z })) addEast(1);
		add_L = convertRtoListPoint(addr_R);
		map_A[add_L.z - 1][add_L.x + 2] = "0";
		map_A[add_L.z    ][add_L.x + 2] = "0";
		map_A[add_L.z + 1][add_L.x + 2] = "0";
		map_A[add_L.z    ][add_L.x + 3] = "0";
		map_A[add_L.z    ][add_L.x + 4] = "0";
		map_A[add_L.z    ][add_L.x + 5] = "0";
		map_A[add_L.z - 1][add_L.x + 4] = "0";
		map_A[add_L.z + 1][add_L.x + 4] = "0";
	}
	else if ((wallset.left == WallType::type16 && wallset.right == WallType::type17) ||
		(wallset.left == WallType::type16 && wallset.right == WallType::type10) ||
		(wallset.left == WallType::type10 && wallset.right == WallType::type17)) {
	return;
}
	else {
		if (!existTile_R({ addr_R.x + 2,addr_R.z })) addEast(1);
		vector<vector<string>> wall = {
			{"-", "-", "-", "-", "-"},
			{"-", "-", "-", "-", "-"},
			{"-", "-", "-", "-", "-"},
			{"-", "-", "-", "-", "-"},
			{"-", "-", "-", "-", "-"},
		};
		paintTile(wall, wallset);
		bool isOnlyFront = isBackClear(wall);
		rotate90Degrees(wall);
		add_L = convertRtoListPoint(addr_R);
		if (isOnlyFront) {
			add_L.x += 4;
			for (int8_t i = -2; i <= 2; i++) {
				for (int8_t j = -2; j <= 0; j++) {
					if (map_A[add_L.z + i][add_L.x + j] == "-" || map_A[add_L.z + i][add_L.x + j] == "0" || map_A[add_L.z + i][add_L.x + j] == "1") {
						if (wall[i + 2][j + 2] != "-") write_map_A(add_L.x + j, add_L.z + i, wall[i + 2][j + 2]);
					}
				}
			}
		}
		else {
			if (!existTile_R2({ addr_R.x + 2,addr_R.z })) addEast(1);
			add_L = convertRtoListPoint(addr_R);
			drawTile(wall, { add_L.x + 4,add_L.z });
		}
	}
}

void Map::rotate90Degrees(vector<vector<string>>& arr) {
	int n = (int)arr.size();
	vector<vector<string>> rotated(n, vector<string>(n));
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < n; ++j) {
			rotated[j][n - 1 - i] = arr[i][j];
		}
	}
	arr = rotated;
}

void Map::paintTile(vector<vector<string>>& tile, const WallSet& wallset) {
	switch (wallset.center) {
	case WallType::center_o:
		tile[0][2] = "1";
		tile[1][2] = "1";
		tile[2][2] = "1";
		break;
	case WallType::center_s:
		tile[2][2] = "1";
		tile[3][2] = "1";
		tile[4][2] = "1";
		break;
	}
	switch (wallset.left) {
	case WallType::type0:
		tile[0][0] = "1";
		tile[0][1] = "1";
		tile[0][2] = "1";
		break;
	case WallType::type1:
		tile[0][0] = "1";
		tile[0][1] = "1";
		tile[1][2] = "1";
		tile[2][2] = "1";
		break;
	case WallType::type2:
		tile[0][2] = "1";
		tile[1][2] = "1";
		tile[2][1] = "1";
		tile[2][0] = "1";
		break;
	case WallType::type3:
		tile[0][0] = "1";
		tile[1][0] = "1";
		tile[2][1] = "1";
		tile[2][2] = "1";
		break;
	case WallType::type4:
		tile[0][2] = "1";
		tile[0][1] = "1";
		tile[1][0] = "1";
		tile[2][0] = "1";
		break;
	case WallType::type5:
		tile[2][0] = "1";
		tile[2][1] = "1";
		tile[2][2] = "1";
		break;
	case WallType::type6:
		tile[2][0] = "1";
		tile[2][1] = "1";
		tile[3][2] = "1";
		tile[4][2] = "1";
		break;
	case WallType::type7:
		tile[2][2] = "1";
		tile[3][2] = "1";
		tile[4][0] = "1";
		tile[4][1] = "1";
		break;
	case WallType::type8:
		tile[2][0] = "1";
		tile[3][0] = "1";
		tile[4][1] = "1";
		tile[4][2] = "1";
		break;
	case WallType::type9:
		tile[4][0] = "1";
		tile[3][0] = "1";
		tile[2][1] = "1";
		tile[2][2] = "1";
		break;
	case WallType::type10:
		tile[4][0] = "1";
		tile[4][1] = "1";
		tile[4][2] = "1";
		break;
	}
	switch (wallset.right) {
	case WallType::type0:
		tile[0][2] = "1";
		tile[0][3] = "1";
		tile[0][4] = "1";
		break;
	case WallType::type1:
		tile[0][2] = "1";
		tile[0][3] = "1";
		tile[1][4] = "1";
		tile[2][4] = "1";
		break;
	case WallType::type2:
		tile[0][4] = "1";
		tile[1][4] = "1";
		tile[2][3] = "1";
		tile[2][2] = "1";
		break;
	case WallType::type3:
		tile[0][2] = "1";
		tile[1][2] = "1";
		tile[2][3] = "1";
		tile[2][4] = "1";
		break;
	case WallType::type4:
		tile[0][4] = "1";
		tile[0][3] = "1";
		tile[1][2] = "1";
		tile[2][2] = "1";
		break;
	case WallType::type5:
		tile[2][4] = "1";
		tile[2][3] = "1";
		tile[2][2] = "1";
		break;
	case WallType::type6:
		tile[2][2] = "1";
		tile[2][3] = "1";
		tile[3][4] = "1";
		tile[4][4] = "1";
		break;
	case WallType::type7:
		tile[2][4] = "1";
		tile[3][4] = "1";
		tile[4][2] = "1";
		tile[4][3] = "1";
		break;
	case WallType::type8:
		tile[2][2] = "1";
		tile[3][2] = "1";
		tile[4][3] = "1";
		tile[4][4] = "1";
		break;
	case WallType::type9:
		tile[2][4] = "1";
		tile[2][3] = "1";
		tile[3][2] = "1";
		tile[4][2] = "1";
		break;
	case WallType::type10:
		tile[4][2] = "1";
		tile[4][3] = "1";
		tile[4][4] = "1";
		break;
	}
	if (tile[4][1] == "-" && wallset.left != WallType::type16) tile[4][1] = "0";
	if (tile[4][2] == "-" && wallset.left != WallType::type16 && wallset.right != WallType::type17) tile[4][2] = "0";
	if (tile[4][3] == "-" && wallset.right != WallType::type16) tile[4][3] = "0";

	// 空白うめうめ
	// 0,1列
	if (tile[0][1] == "1") {
		if (tile[2][1] == "-") tile[2][1] = "0";
		if (tile[4][1] == "-") tile[4][1] = "0";
	}
	// 0,2列
	if (tile[0][2] == "1") {
		if (tile[1][2] == "-") tile[1][2] = "0";
		if (tile[2][2] == "-") tile[2][2] = "0";
		if (tile[3][2] == "-") tile[3][2] = "0";
		if (tile[4][2] == "-") tile[4][2] = "0";
	}
	else if (tile[1][2] == "1") {
		if (tile[2][2] == "-") tile[2][2] = "0";
		if (tile[3][2] == "-") tile[3][2] = "0";
		if (tile[4][2] == "-") tile[4][2] = "0";
	}
	else if (tile[2][2] == "1") {
		if (tile[3][2] == "-") tile[3][2] = "0";
		if (tile[4][2] == "-") tile[4][2] = "0";
	}

	// 0,3列
	if (tile[0][3] == "1") {
		if (tile[2][3] == "-") tile[2][3] = "0";
		if (tile[4][3] == "-") tile[4][3] = "0";
	}
}

void Map::drawTile(vector<vector<string>>& tile, MapAddress add_L){
	for (int i = 0; i < 5; ++i) {
		for (int j = 0; j < 5; ++j) {
			if (map_A[add_L.z - 2 + i][add_L.x - 2 + j] == "-" && (tile[i][j] == "1" || tile[i][j] == "0")) {
				if (tile[i][j] != "-") write_map_A(add_L.x - 2 + j, add_L.z - 2 + i, tile[i][j]);
			}
		}
	}
}

// タイルの後ろ半分に壁があったらfalse、なかったらtrueを返す
bool Map::isBackClear(vector<vector<string>>& tile){ 
	for (const auto& row : tile[0]) {
		if (row == "1" || row == "0") return false;
	}
	for (const auto& row : tile[1]) {
		if (row == "1" || row == "0") return false;
	}
	return true;
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
	//cout << add_L.x << " " << add_L.z << endl;
 //	cout << map_A[add_L.z][add_L.x + 2] << endl;
	//cout << map_A[add_L.z][add_L.x - 2] << endl;
	//cout << map_A[add_L.z - 2][add_L.x] << endl;
	//cout << map_A[add_L.z + 2][add_L.x] << endl;
	if (abs(angle - 90) < 5) {
		frontWall = condition_getAroundWallState(add_L.x + 2, add_L.z);
		backWall = condition_getAroundWallState( add_L.x - 2, add_L.z);
		leftWall = condition_getAroundWallState( add_L.x, add_L.z - 2);
		rightWall = condition_getAroundWallState(add_L.x, add_L.z + 2);
	}
	else if (abs(angle - 180) < 5) {
		frontWall = condition_getAroundWallState(add_L.x, add_L.z - 2);
		backWall  = condition_getAroundWallState(add_L.x, add_L.z + 2);
		leftWall  = condition_getAroundWallState(add_L.x - 2, add_L.z);
		rightWall = condition_getAroundWallState(add_L.x + 2, add_L.z);
	}
	else if (abs(angle - 270) < 5) {
		frontWall = condition_getAroundWallState(add_L.x - 2, add_L.z);
		backWall  = condition_getAroundWallState(add_L.x + 2, add_L.z);
		leftWall  = condition_getAroundWallState(add_L.x, add_L.z + 2);
		rightWall = condition_getAroundWallState(add_L.x, add_L.z - 2);
	}
	else if ((angle >= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
		frontWall = condition_getAroundWallState(add_L.x, add_L.z + 2);
		backWall  = condition_getAroundWallState(add_L.x, add_L.z - 2);
		leftWall  = condition_getAroundWallState(add_L.x + 2, add_L.z);
		rightWall = condition_getAroundWallState(add_L.x - 2, add_L.z);
	}
}

void Map::replaceLineTo0() {
	for (auto& row : map_A) {
		std::replace(row.begin(), row.end(), "-", "0");
	}
}

TileState Map::getTileState(MapAddress addr_R, int16_t relative_angle) {
	if (!existTile_R(addr_R)) {
		//cout << "tile not found " << addr_R.x << " " << addr_R.z << ", " << relative_angle << endl;
		return TileState::UNKNOWN;
	}
	MapAddress addr_L = convertRtoListPoint(addr_R);
	
	if (relative_angle == -1) {
		return TileStateMap2[map_A[addr_L.z - 1][addr_L.x - 1]];
	}
	return TileStateMap2[map_A[addr_L.z + 1][addr_L.x + 1]];
}

void Map::getAroundTileState(MapAddress addr_R, TileState& front, TileState& back, TileState& left, TileState& right, double angle) {
	if (angle == -1) {
		angle = gyro.getGyro();
	}
	//cout << "angle = " << angle << endl;
	if (abs(angle - 90) < 5) {
		front = getTileState({ addr_R.x + 2, addr_R.z }, -1);
		back = getTileState({ addr_R.x - 2, addr_R.z }, 1);
		left = getTileState({ addr_R.x, addr_R.z - 2 }, 1);
		right = getTileState({ addr_R.x, addr_R.z + 2 }, -1);
	}
	else if (abs(angle - 180) < 5) {
		front = getTileState({ addr_R.x, addr_R.z - 2 }, 1);
		back = getTileState({ addr_R.x, addr_R.z + 2 }, -1);
		left = getTileState({ addr_R.x - 2, addr_R.z }, 1);
		right = getTileState({ addr_R.x + 2, addr_R.z }, -1);
	}
	else if (abs(angle - 270) < 5) {
		front = getTileState({ addr_R.x - 2, addr_R.z }, 1);
		back = getTileState({ addr_R.x + 2, addr_R.z }, -1);
		left = getTileState({ addr_R.x, addr_R.z + 2 }, -1);
		right = getTileState({ addr_R.x, addr_R.z - 2 }, 1);
	}
	else if ((angle >= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
		front = getTileState({ addr_R.x, addr_R.z + 2 }, -1);
		back = getTileState({ addr_R.x, addr_R.z - 2 }, 1);
		left = getTileState({ addr_R.x + 2, addr_R.z }, -1);
		right = getTileState({ addr_R.x - 2, addr_R.z }, 1);
	}
	else cout << "unreliable angle in getAroundTileState, " << angle << endl;
	//cout << "front = " << (int)front << " back = " << (int)back << " left = " << (int)left << " right = " << (int)right << endl;
}