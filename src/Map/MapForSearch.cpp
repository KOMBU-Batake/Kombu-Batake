#include "MapForSearch.h"

void MapperS::updatePostion(const int& dx_R, const int& dz_R)
{
	currentTile_R.x += dx_R;
	currentTile_R.z += dz_R;
	currentTile_A.x += dx_R;
	currentTile_A.z += dz_R;
}

void MapperS::addNorth(const int& i) { // 上に追加
	// map_Sの下に4i行追加
	//cout << "map_S.size() = " << map_S.size() << endl;
	map_S.resize(map_S.size() + 4 * i, vector<int>(map_S[0].size(), 0));
	// map_Sの要素を下に4iずつずらす
	rotate(map_S.rbegin(), map_S.rbegin() + 4 * i, map_S.rend());
	startTile_A.z += 2;
	currentTile_A.z += 2;
}

void MapperS::addSouth(const int& i) { // 下に追加
	// map_Sの下に1行追加
	//cout << "map_S.size() = " << map_S.size() << endl;
	map_S.resize(map_S.size() + 4 * i, vector<int>(map_S[0].size(), 0));
}

void MapperS::addWest(const int& j) { // 左に追加
	// 既存の行に新しい列を追加
	for (int i = 0; i < map_S.size(); ++i) {
		map_S[i].resize(map_S[i].size() + 4 * j, 0);
	}
	// 全体の要素を左に1つずらす
	for (int i = 0; i < map_S.size(); ++i) {
		rotate(map_S[i].rbegin(), map_S[i].rbegin() + 4 * j, map_S[i].rend());
	}
	startTile_A.x += 2;
	currentTile_A.x += 2;
}

void MapperS::addEast(const int& j) { // 右に追加
	// 既存の行に新しい列を追加
	for (int i = 0; i < map_S.size(); ++i) {
		map_S[i].resize(map_S[i].size() + 4 * j, 0);
	}
}

void MapperS::resetSearchPoint() { // 探索用のマスを-1でリセット
	for (int i = 0; i < map_S.size(); i+=2) {
		for (int j = 0; j < map_S[i].size(); j+=2) {
			map_S[i][j] = -1;
		}
	}
}

vector<MapAddress> MapperS::getRoute(MapAddress& Goal_R, const vector<MapAddress>& stack_of_DFS)
{
	MapAddress start_A = currentTile_A;
	MapAddress start_L = convertAtoListPoint(start_A);
	MapAddress goal_A = convertRtoA(Goal_R);
	MapAddress goal_L = convertAtoListPoint(goal_A);

	// まずはスタックベースで経路を探す
  auto goal_it = find(stack_of_DFS.rbegin(), stack_of_DFS.rend(), Goal_R); // 直近で通ったGoalを探す
	if (goal_it != stack_of_DFS.rend()) {
		++goal_it; // イテレータをインクリメント
	}
	vector<MapAddress> route_stack(distance(stack_of_DFS.rbegin(),goal_it));
	copy(stack_of_DFS.rbegin(), goal_it, route_stack.begin());

	vector<MapAddress> stack_based_route_R = { route_stack[0]};
	for (auto it = route_stack.begin() + 1; it != route_stack.end() - 1; ++it) {
		auto it_second_tile = find(it+1, route_stack.end(), *it);
		stack_based_route_R.push_back(*it);
		if (it_second_tile != route_stack.end()) { // 見つかった
			it = it_second_tile;
		}
	}
	stack_based_route_R.push_back(route_stack.back());

	// 幅優先探索
	resetSearchPoint();
	deque<deque<MapAddress>> queue_L = { {start_L} };
	map_S[start_L.z][start_L.x] = 1;
	int count = 2;
	while (!queue_L.empty()) {
		deque<MapAddress> points = queue_L.front(); // キューの先頭を取得
		queue_L.pop_front();
		deque<MapAddress> stack_to_add;
		for (auto& point : points) {
			// 8方向に探索
			deque<MapAddress> canGo = canGoNEWS(point, count);
			for (auto& next : canGo) {
				if (next == goal_L) {
					// 経路を見つけた
					vector<MapAddress> route_BFS_R;
					route_BFS_R.push_back(Goal_R);
					for (auto& k : map_S) {
						for (auto& l : k) {
							cout << l << " ";
						}
						cout << endl;
					}
					while (next != start_L) {
						// 逆からたどる
						count--;
						deque<MapAddress> canGo = canGoNEWS(next, -1, count, false);
						cout << "size: " << canGo.size() << ", count: " << count << ", center: " << next.x << " " << next.z << ", " << map_S[next.z][next.x] << endl;
						next = canGo[0];
						route_BFS_R.push_back(convertListPointtoR(next));
					}
					reverse(route_BFS_R.begin(), route_BFS_R.end());
					for (auto& k : route_BFS_R) {
						cout << k.x << " " << k.z << endl;
					}
					if (stack_based_route_R.size() < route_BFS_R.size()) {
						return stack_based_route_R;
					} else return route_BFS_R;
				}
				stack_to_add.push_back(next);
			}
		}
		queue_L.push_back(stack_to_add);
		count++;
	}

	return vector<MapAddress>();
}

void MapperS::markAroundStatus(const aroundStatus& status, const double& angle)
{
	MapAddress currentTile_L = convertAtoListPoint(startTile_A);
	vector<bool> around4 = { status.front, status.left, status.back, status.right };
	if (abs(angle - 90) < 5) {
		rotate(around4.begin(), around4.begin() + 1, around4.end());
		map_S[currentTile_L.z - 1][currentTile_L.x + 1] = status.front_left;
		map_S[currentTile_L.z + 1][currentTile_L.x + 1] = status.front_right;
	}
	else if (abs(angle - 180) < 5) {
		// そのまま
		map_S[currentTile_L.z - 1][currentTile_L.x - 1] = status.front_left;
		map_S[currentTile_L.z - 1][currentTile_L.x + 1] = status.front_right;
	}
	else if (abs(angle - 270) < 5) {
		rotate(around4.rbegin(), around4.rbegin() + 1, around4.rend());
		map_S[currentTile_L.z + 1][currentTile_L.x - 1] = status.front_left;
		map_S[currentTile_L.z - 1][currentTile_L.x - 1] = status.front_right;
	}
	else if ((angle >= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
		rotate(around4.begin(), around4.begin() + 2, around4.end());
		map_S[currentTile_L.z + 1][currentTile_L.x + 1] = status.front_left;
		map_S[currentTile_L.z + 1][currentTile_L.x - 1] = status.front_right;
	}
	map_S[currentTile_L.z - 1][currentTile_L.x] = around4[0];
	map_S[currentTile_L.z][currentTile_L.x + 1] = around4[3];
	map_S[currentTile_L.z + 1][currentTile_L.x] = around4[2];
	map_S[currentTile_L.z][currentTile_L.x - 1] = around4[1];
}

/* 探索候補探しと、数字の書き込み
 * 既に書き込んである場合は書き込まない */
deque<MapAddress> MapperS::canGoNEWS(const MapAddress& addr_A, const int& num, const int& cond, bool cond2) {
	deque<MapAddress> returnVector;
	// 上
	if (addr_A.z - 1 >= 0 && map_S[addr_A.z - 1][addr_A.x] &&
		map_S[addr_A.z - 2][addr_A.x] == cond)
	{
		returnVector.push_back({ addr_A.x, addr_A.z - 2 });
		if (cond2) map_S[addr_A.z - 2][addr_A.x] = num;
	}
	// 右上
	if (addr_A.z - 1 >= 0 && addr_A.x + 1 < map_S[0].size() && map_S[addr_A.z - 1][addr_A.x + 1] &&
		map_S[addr_A.z - 2][addr_A.x + 2] == cond)
	{
		returnVector.push_back({ addr_A.x + 2, addr_A.z - 2 });
		if (cond2) map_S[addr_A.z - 2][addr_A.x + 2] = num;
	}
	// 右
	if (addr_A.x + 1 < map_S[0].size() && map_S[addr_A.z][addr_A.x + 1] &&
		map_S[addr_A.z][addr_A.x + 2] == cond)
	{
		returnVector.push_back({ addr_A.x + 2, addr_A.z });
		if (cond2) map_S[addr_A.z][addr_A.x + 2] = num;
	}
	// 右下
	if (addr_A.z + 1 < map_S.size() && addr_A.x + 1 < map_S[0].size() && map_S[addr_A.z + 1][addr_A.x + 1] &&
		map_S[addr_A.z + 2][addr_A.x + 2] == cond)
	{
		returnVector.push_back({ addr_A.x + 2, addr_A.z + 2 });
		if (cond2) map_S[addr_A.z + 2][addr_A.x + 2] = num;
	}
	// 下
	if (addr_A.z + 1 < map_S.size() && map_S[addr_A.z + 1][addr_A.x] &&
		map_S[addr_A.z + 2][addr_A.x] == cond)
	{
		returnVector.push_back({ addr_A.x, addr_A.z + 2 });
		if (cond2) map_S[addr_A.z + 2][addr_A.x] = num;
	}
	// 左下
	if (addr_A.z + 1 < map_S.size() && addr_A.x - 1 >= 0 && map_S[addr_A.z + 1][addr_A.x - 1] &&
		map_S[addr_A.z + 2][addr_A.x - 2] == cond)
	{
		returnVector.push_back({ addr_A.x - 2, addr_A.z + 2 });
		if (cond2) map_S[addr_A.z + 2][addr_A.x - 2] = num;
	}
	// 左
	if (addr_A.x - 1 >= 0 && map_S[addr_A.z][addr_A.x - 1] &&
		map_S[addr_A.z][addr_A.x - 2] == cond)
	{
		returnVector.push_back({ addr_A.x - 2, addr_A.z });
		if (cond2) map_S[addr_A.z][addr_A.x - 2] = num;
	}
	// 左上
	if (addr_A.z - 1 >= 0 && addr_A.x - 1 >= 0 && map_S[addr_A.z - 1][addr_A.x - 1] &&
		map_S[addr_A.z - 2][addr_A.x - 2] == cond)
	{
		returnVector.push_back({ addr_A.x - 2, addr_A.z - 2 });
		if (cond2) map_S[addr_A.z - 2][addr_A.x - 2] = num;
	}

	return returnVector;
}