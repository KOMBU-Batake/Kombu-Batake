#include "Map2.h"

void Map2::addNorth(const int i) { // 上に追加
	// map_Aの下に4i行追加
	//cout << "map_A.size() = " << map_A.size() << endl;
	map_A.resize(map_A.size() + 4 * i, vector<string>(map_A[0].size(), "-"));
	// map_Aの要素を下に4iずつずらす
	rotate(map_A.rbegin(), map_A.rbegin() + 4 * i, map_A.rend());
	startTile_A.z++;
	currentTile_A.z++;
	left_top_R.z--;
}

void Map2::addSouth(const int i) { // 下に追加
	// map_Aの下に1行追加
	//cout << "map_A.size() = " << map_A.size() << endl;
	map_A.resize(map_A.size() + 4 * i, vector<string>(map_A[0].size(), "-"));
	right_bottom_R.z += 1;
}

void Map2::addWest(const int j) { // 左に追加
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

void Map2::addEast(const int j) { // 右に追加
	// 既存の行に新しい列を追加
	for (int i = 0; i < map_A.size(); ++i) {
		map_A[i].resize(map_A[i].size() + 4 * j, "-");
	}
	right_bottom_R.x++;
}