#include "DFS.h"

void DFS() {
	vector<stackDFS> stack;
	vector<MapAddress> footprints = {mapper.currentTile_R};
	int tail = -1;
	
	mapper.printMap();
	while (robot->step(timeStep) != -1) {
		// 角度(方位)を取得
		double angle = gyro.getGyro();

		// 壁の情報を取得
		WallSet front_mp = lidar2.getWallType(LiDAR_degree::FRONT);
		WallSet back_mp = lidar2.getWallType(LiDAR_degree::BACK);
		WallSet left_mp = lidar2.getWallType(LiDAR_degree::LEFT);
		WallSet right_mp = lidar2.getWallType(LiDAR_degree::RIGHT);
		// 壁を提出用マップに登録
		if (abs(angle - 90) < 5) {
			mapper.markAroundWall(left_mp, right_mp, back_mp, front_mp);
		}
		else if (abs(angle - 180) < 5) {
			mapper.markAroundWall(front_mp, back_mp, left_mp, right_mp);
		}
		else if (abs(angle - 270) < 5) {
			mapper.markAroundWall(right_mp, left_mp, front_mp, back_mp);
		}
		else if ((angle >= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
			mapper.markAroundWall(back_mp, front_mp, right_mp, left_mp);
		}
		mapper.printMap();

		// 進める方向を探す
		canGoSet cango_set = lidar2.identifyCanGo();
		// 斜めの選択肢を吟味
		if ((int)cango_set.front < 3 || (int)cango_set.left < 3) cango_set.front_left = canGo::NO;
		if ((int)cango_set.front < 3 || (int)cango_set.right < 3) cango_set.front_right = canGo::NO;
		// 探索用マップに記録
		mapperS.markAroundStatus({ (int)cango_set.front < 3,
															 (int)cango_set.left < 3,
															 (int)cango_set.right < 3,
															 (int)cango_set.back < 3,
															 (int)cango_set.front_left < 3,
															 (int)cango_set.front_right < 3 }, 
															 angle);

		// タイルの情報を取得
		vector<TileState> front_tile(2), back_tile(2), left_tile(2), right_tile(2), front_left_tile(2), front_right_tile(2);
		front_tile = mapper.getTileStateLR(mapper.currentTile_R, angle, LiDAR_degree::FRONT);
		back_tile = mapper.getTileStateLR(mapper.currentTile_R, angle, LiDAR_degree::BACK);
		left_tile = mapper.getTileStateLR(mapper.currentTile_R, angle, LiDAR_degree::LEFT);
		right_tile = mapper.getTileStateLR(mapper.currentTile_R, angle, LiDAR_degree::RIGHT);
		front_left_tile = mapper.getTileStateLR(mapper.currentTile_R, angle, LiDAR_degree::FRONT_LEFT);
		front_right_tile = mapper.getTileStateLR(mapper.currentTile_R, angle, LiDAR_degree::FRONT_RIGHT);

		// 進行方向の選択
		canGo canGoFront = judgeCanGo(front_tile, (int)cango_set.front < 3);
		canGo canGoBack = judgeCanGo(back_tile, (int)cango_set.back < 3);
		canGo canGoLeft = judgeCanGo(left_tile, (int)cango_set.left < 3);
		canGo canGoRight = judgeCanGo(right_tile, (int)cango_set.right < 3);
		canGo canGoFrontLeft = judgeCanGo(front_left_tile, (int)cango_set.front_left < 3);
		canGo canGoFrontRight = judgeCanGo(front_right_tile, (int)cango_set.front_right < 3);

		bool hasStack = addToStackDFS(stack, angle, canGoFront, canGoLeft, canGoRight, canGoBack, canGoFrontLeft, canGoFrontRight);
		MapAddress nextTile = pickSatckDFS(stack);

		// 進む
		vector<MapAddress> route = mapperS.getRoute(nextTile, footprints);
		for (auto& tile : route) {
			if (tile == *route.begin()) continue;

		}

		//// 被災者
		//mapper.printMap();
		//cout << "stack size. " << stack.size() << " ========================================" << endl;
		//bool isHole = false;
		//if (direction_of_travel == NEWS::NORTH) {
		//	isHole = tank.gpsTrace(gps.moveTiles(0, -1), 4.5);
		//}
		//else  if (direction_of_travel == NEWS::EAST) {
		//	isHole = tank.gpsTrace(gps.moveTiles(1, 0), 4.5);
		//}
		//else if (direction_of_travel == NEWS::SOUTH) {
		//	isHole = tank.gpsTrace(gps.moveTiles(0, 1), 4.5);
		//}
		//else if (direction_of_travel == NEWS::WEST) {
		//	isHole = tank.gpsTrace(gps.moveTiles(-1, 0), 4.5);
		//}
		//else if (direction_of_travel == NEWS::NO) {
		//	cout << "No way to go" << endl;
		//	break;
		//}

		//angle = gyro.getGyro();
		//if (!isHole) {
		//	cout << "Hole is there" << endl;
		//	HoleIsThere(angle);
		//}
		//else {
		//	mapper.updatePostion(angle);
		//	// 現在のタイルの情報を取得、記録する
		//	colorsensor.update();
		//	TileState col = colorsensor.getTileColor();
		//	TileState recordedTile = mapper.getTileState(mapper.currentTile_R);
		//	if (recordedTile != TileState::START){
		//		mapper.markTileAs(mapper.currentTile_R, col, angle);
		//	}
		//	if (col == TileState::AREA1to4 || col == TileState::AREA3to4) {
		//		Area4IsThere(angle, tail, stack,dontStack);
		//	}
		//	else {
		//		// スタックに現在のタイルを追加
		//		if (!dontStack) stack.push_back(mapper.currentTile_R);
		//	}
		//}
	}
	mapper.replaceLineTo0();
	mapper.printMap();
	sendMap(mapper.map_A);
}

MapAddress pickSatckDFS(vector<stackDFS>& stack)
{
	stackDFS top = stack.back();
	MapAddress next;
	// 斜め
	if (top.diagonal.size() != 0) {
		next = top.diagonal[0];
		stack.back().diagonal.pop_back();
	}
	else if (top.Go.size() != 0) {
		next = top.Go.back();
		stack.back().Go.pop_back();
	}
	else if (top.PartlyVisited.size() != 0) {
		next = top.PartlyVisited.back();
		stack.back().PartlyVisited.pop_back();
	}
	else {
		stack.pop_back();
		return pickSatckDFS(stack); // なかったら再起
	}
	return next;
}

canGo judgeCanGo(const vector<TileState>& tileState, const bool canGo)
{
	if (canGo) {
		if (find(tileState.begin(), tileState.end(), TileState::UNKNOWN) != tileState.end()) { // 未探索
			if (all_of(tileState.begin(), tileState.end(), [](const TileState& state) { return state == TileState::UNKNOWN; })) {
				return canGo::GO;
			}
			return canGo::PARTLY_VISITED;
		}
		else if (any_of(tileState.begin(), tileState.end(), [](const TileState& state) { return state == TileState::HOLE || state == TileState::AREA3to4 || state == TileState::AREA1to4; })) { // 穴かエリア4がある {
			return canGo::NO;
		}
		else { // 何かしらのタイルを探索済み
			return canGo::VISITED;
		}
	}
	else {
		return canGo::NO;
	}
}

bool addToStackDFS(vector<stackDFS>& stack, const double& angle, const canGo& front, const canGo& left, const canGo& right, const canGo& back, const canGo& front_left, const canGo& front_right)
{
	stackDFS stackElement;
	stackElement.center = mapper.currentTile_R;
	canGo north = canGo::NO , east = canGo::NO, south = canGo::NO, west = canGo::NO, north_east = canGo::NO, south_east = canGo::NO, south_west = canGo::NO, north_west = canGo::NO;
	if (abs(angle - 90) < 5) {
		north = left; east = front; south = right; west = back, north_east = front_left, south_east = front_right;
	}
	else if (abs(angle - 180) < 5) {
		north = front; east = right; south = back; west = left, north_west = front_left, north_east = front_right;
	}
	else if (abs(angle - 270) < 5) {
		north = right; east = back; south = left; west = front, south_west = front_left, north_west = front_right;
	}
	else if ((angle >= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
		north = back; east = left; south = front; west = right, south_east = front_left, south_west = front_right;
	}

	if (north == canGo::GO) {
		stackElement.Go.push_back({ mapper.currentTile_R.x, mapper.currentTile_R.z - 1 });
	} else if (north == canGo::PARTLY_VISITED) {
		stackElement.PartlyVisited.push_back({ mapper.currentTile_R.x, mapper.currentTile_R.z - 1 });
	}

	if (west == canGo::GO) {
		stackElement.Go.push_back({ mapper.currentTile_R.x - 1, mapper.currentTile_R.z });
	} else 	if (west == canGo::PARTLY_VISITED) {
		stackElement.PartlyVisited.push_back({ mapper.currentTile_R.x - 1, mapper.currentTile_R.z });
	}

	if (east == canGo::GO) {
		stackElement.Go.push_back({ mapper.currentTile_R.x + 1, mapper.currentTile_R.z });
	} else if (east == canGo::PARTLY_VISITED) {
		stackElement.PartlyVisited.push_back({ mapper.currentTile_R.x + 1, mapper.currentTile_R.z });
	}

	if (south == canGo::GO) {
		stackElement.Go.push_back({ mapper.currentTile_R.x, mapper.currentTile_R.z + 1 });
	} else if (south == canGo::PARTLY_VISITED) {
		stackElement.PartlyVisited.push_back({ mapper.currentTile_R.x, mapper.currentTile_R.z + 1 });
	}

	if (north_east == canGo::GO) {
		stackElement.diagonal.push_back({ mapper.currentTile_R.x + 1, mapper.currentTile_R.z - 1 });
	}
	else if (north_west == canGo::GO) {
		stackElement.diagonal.push_back({ mapper.currentTile_R.x - 1, mapper.currentTile_R.z - 1 });
	}
	else if (south_east == canGo::GO) {
		stackElement.diagonal.push_back({ mapper.currentTile_R.x + 1, mapper.currentTile_R.z + 1 });
	}
	else if (south_west == canGo::GO) {
		stackElement.diagonal.push_back({ mapper.currentTile_R.x - 1, mapper.currentTile_R.z + 1 });
	}

	if (stackElement.Go.size() == 0 && stackElement.PartlyVisited.size() == 0 && stackElement.diagonal.size() == 0) {
		return false;
	}
	else {
		stack.push_back(stackElement);
		return true;
	}
}

NEWS searchAround(double angle, int& tail, vector<MapAddress>& stack, bool& dontStack) {
	PotentialDirectionsOfTravel PDoT = { canGo::NO,canGo::NO,canGo::NO,canGo::NO };
	pcLiDAR.update(gps.expectedPos);

	WallSet front_mp,back_mp,left_mp,right_mp;
	TileState front_tile, back_tile, left_tile, right_tile;

	mapper.getAroundTileState(mapper.currentTile_R, front_tile, back_tile, left_tile, right_tile, angle); // マップ上での周囲のタイルの有無を取得
	cout << "front tile: " << (int)front_tile << ", back tile: " << (int)back_tile << ", left tile: " << (int)left_tile << ", right tile: " << (int)right_tile << endl;

	//searchFront(PDoT, front_mp, front_tile); // 前
	//searchBack(PDoT, back_mp, back_tile); // 後ろ
	//searchLeft(PDoT, left_mp, left_tile); // 左
	//searchRight(PDoT, right_mp, right_tile); // 右

	NEWSset directionNEWS = { canGo::NO,canGo::NO,canGo::NO,canGo::NO };
	// 壁をマップに登録
	if (abs(angle - 90) < 5) {
		mapper.markAroundWall(left_mp, right_mp, back_mp, front_mp);
		directionNEWS = { PDoT.left, PDoT.front, PDoT.right, PDoT.back };
	}
	else if (abs(angle - 180) < 5) {
		mapper.markAroundWall(front_mp, back_mp, left_mp, right_mp);
		directionNEWS = { PDoT.front, PDoT.right, PDoT.back, PDoT.left };
	}
	else if (abs(angle - 270) < 5) {
		mapper.markAroundWall(right_mp, left_mp, front_mp, back_mp);
		directionNEWS = { PDoT.right, PDoT.back, PDoT.left, PDoT.front };
	}
	else if ((angle >= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
		mapper.markAroundWall(back_mp, front_mp, right_mp, left_mp);
		directionNEWS = { PDoT.back, PDoT.left, PDoT.front, PDoT.right };
	}
	std::cout << "front: " << (int)PDoT.front << " back: " << (int)PDoT.back << " left: " << (int)PDoT.left << " right: " << (int)PDoT.right << endl;

	// 進行方向の選択
	// 北 -> 東 -> 南 -> 西 の順に優先する
	if (directionNEWS.north == canGo::GO) {
		return NEWS::NORTH;
	}
	else if (directionNEWS.east == canGo::GO) {
		return NEWS::EAST;
	}
	else if (directionNEWS.south == canGo::GO) {
		return NEWS::SOUTH;
	}
	else if (directionNEWS.west == canGo::GO) {
		return NEWS::WEST;
	}
	else { // スタックを拾っていく
		int tmp = (int)stack.size() - 2;
		if (tmp < 0) {
			cout << "reach 0" << endl;
			return NEWS::NO;
		}
		MapAddress nextTile = stack[tmp];
		cout << "tail; " << tail << endl;
		dontStack = true;
		stack.pop_back();
		if (nextTile.x > mapper.currentTile_R.x) {
			return NEWS::EAST;
		}
		else if (nextTile.x < mapper.currentTile_R.x) {
			return NEWS::WEST;
		}
		else if (nextTile.z > mapper.currentTile_R.z) {
			return NEWS::SOUTH;
		}
		else if (nextTile.z < mapper.currentTile_R.z) {
			return NEWS::NORTH;
		}
		else {
			return NEWS::NO;
		}
	}
}

/* 手前に半マス進めるか否か
   直径は余裕を持って8cmで勘定する */
bool condirtion_canGo(const WallSet& wallset) 
{
	uint8_t left = static_cast<uint8_t>(wallset.left);
	uint8_t right = static_cast<uint8_t>(wallset.right);
	uint8_t center = static_cast<uint8_t>(wallset.center);
	if ((left <= 5U || left == 9U || left == 11U) &&
			(right <= 5U || right == 6U || right == 11U) &&
			 center >= 13U) {
		return true;
	}
	return false;
}

void HoleIsThere(const double& angle)
{
	if (abs(angle - 90) < 5) {
		mapper.markTileAs({ mapper.currentTile_R.x + 2,mapper.currentTile_R.z }, TileState::HOLE, angle);
	}
	else if (abs(angle - 180) < 5) {
		mapper.markTileAs({ mapper.currentTile_R.x,mapper.currentTile_R.z - 2 }, TileState::HOLE, angle);
	}
	else if (abs(angle - 270) < 5) {
		mapper.markTileAs({ mapper.currentTile_R.x - 2,mapper.currentTile_R.z }, TileState::HOLE, angle);
	}
	else if ((angle >= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
		mapper.markTileAs({ mapper.currentTile_R.x,mapper.currentTile_R.z + 2 }, TileState::HOLE, angle);
	}
	tank.gpsTrace(gps.last_expectedPos,4);
	gps.returnTolastPos();
}

void Area4IsThere(const double& angle, int tail, vector<MapAddress>& stack, bool& dontStack) // チェックポイントタイルだけ踏んでやるHAHAHA
{
	if (abs(angle - 90) < 5) {
		searchAround(angle, tail, stack, dontStack);
		tank.gpsTrace(gps.moveTiles(-1, 0), 5);
		double angleN = gyro.getGyro();
		mapper.updatePostion(angleN);
	}
	else if (abs(angle - 180) < 5) {
		searchAround(angle, tail, stack, dontStack);
		tank.gpsTrace(gps.moveTiles(0, 1), 5);
		double angleN = gyro.getGyro();
		mapper.updatePostion(angleN);
	}
	else if (abs(angle - 270) < 5) {
		searchAround(angle, tail, stack, dontStack);
		tank.gpsTrace(gps.moveTiles(1, 0), 5);
		double angleN = gyro.getGyro();
		mapper.updatePostion(angleN);
	}
	else if ((angle >= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
		searchAround(angle, tail, stack, dontStack);
		tank.gpsTrace(gps.moveTiles(0, -1), 5);
		double angleN = gyro.getGyro();
		mapper.updatePostion(angleN);
	}
}

void sendMap(vector<vector<string>>& map) { // mallocを使ってるのが良くないね
	int width = (int)map.size();
	int height = (int)map[0].size();
	string flattened = "";
	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {

			flattened += map[i][j] + ","; // Flatten the array with comma separators
		}
	}

	flattened.pop_back(); // Remove the last unnecessary comma

	char *message=(char*)malloc(8 + flattened.size()); // Allocate memory for the message

	memcpy(message, &width, sizeof(width)); // The first 2 integers in the message array are width, height
	memcpy(&message[4], &height, sizeof(height));

	memcpy(&message[8], flattened.c_str(), flattened.size()); // Copy in the flattened map afterwards

	emitter->send(message, 8+(int)flattened.size()); // Send map data

	char msg = 'M'; // Send map evaluate request
	emitter->send(&msg, sizeof(msg));

	while (robot->step(timeStep) != -1) {
		msg = 'E'; // Send an Exit message to get Map Bonus
		emitter->send(&msg, sizeof(msg));
	}
}