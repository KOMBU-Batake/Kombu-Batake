#include "DFS.h"

void DFS() {
	vector<MapAddress> stack = {mapper.currentTile_R};
	int tail = -1;

	mapper.printMap();
	while (robot->step(timeStep) != -1) {
		cout << "A; " << mapper.currentTile_A.x << " " << mapper.currentTile_A.z << endl;
		cout << "R; " << mapper.currentTile_R.x << " " << mapper.currentTile_R.z << endl;
		// 進行方向の選択
		double angle = gyro.getGyro();
		NEWS direction_of_travel = searchAround(angle);
		mapper.printMap();
		cout << "========================================" << endl;
		bool isHole = false;
		if (direction_of_travel == NEWS::NORTH) {
			isHole = tank.gpsTrace(gps.moveTiles(0, -1), 4);
		}
		else  if (direction_of_travel == NEWS::EAST) {
			isHole = tank.gpsTrace(gps.moveTiles(1, 0), 4);
		}
		else if (direction_of_travel == NEWS::SOUTH) {
			isHole = tank.gpsTrace(gps.moveTiles(0, 1), 4);
		}
		else if (direction_of_travel == NEWS::WEST) {
			isHole = tank.gpsTrace(gps.moveTiles(-1, 0), 4);
		}
		else if (direction_of_travel == NEWS::NO) {
			cout << "No way to go" << endl;
			while (robot->step(timeStep) != -1);
		}

		angle = gyro.getGyro();
		if (!isHole) {
			cout << "Hole is there" << endl;
			HoleIsThere(angle);
		}
		else {
			mapper.updatePostion(angle);
			// 現在のタイルの情報を取得、記録する
			colorsensor.update();
			TileState col = colorsensor.getTileColor();
			mapper.markTileAs(mapper.currentTile_R, col);
			if (col == TileState::AREA1to4 || col == TileState::AREA3to4) {
				Area4IsThere(angle);
			}
			else {
				// スタックに現在のタイルを追加
			}
		}
		
	}
}

NEWS searchAround(double angle) {
	PotentialDirectionsOfTravel PDoT = { canGo::NO,canGo::NO,canGo::NO,canGo::NO };
	pcLiDAR.update(gps.expectedPos);

	WallSet front_mp,back_mp,left_mp,right_mp;
	TileState front_tile, back_tile, left_tile, right_tile;

	mapper.getAroundTileState(mapper.currentTile_R, front_tile, back_tile, left_tile, right_tile, angle); // マップ上での周囲のタイルの有無を取得

	searchFront(PDoT, front_mp, front_tile); // 前
	searchBack(PDoT, back_mp, back_tile); // 後ろ
	searchLeft(PDoT, left_mp, left_tile); // 左
	searchRight(PDoT, right_mp, right_tile); // 右

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
	else if ((angle <= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
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
		return NEWS::NO;
	}
}

void searchFront(PotentialDirectionsOfTravel& PDoT, WallSet& front_mp, const TileState& front_tile)
{
	WallSet front = pcLiDAR.identifyWall(LiDAR_degree::FRONT);
	std::cout << "front: " << (int)front.left << " " << (int)front.center << " " << (int)front.right;
	if ((front.left == WallType::type0 || front.left == WallType::typeNo) && front.center == WallType::center_n && (front.right == WallType::type0 || front.right == WallType::typeNo)) {
		if (front_tile == TileState::UNKNOWN) { // 未探索
			PDoT.front = canGo::GO;
		}
		else if (front_tile == TileState::HOLE || front_tile == TileState::AREA3to4 || front_tile == TileState::AREA1to4) {
			PDoT.front = canGo::NO;
		}
		else { // 何かしらのタイルを探索済み
			PDoT.front = canGo::VISITED;
		}
	}
	else {
		PDoT.front = canGo::NO;
	}
	front_mp = front;
}

void searchBack(PotentialDirectionsOfTravel& PDoT, WallSet& back_mp, const TileState& back_tile) // ToDo: 基本的にWallState::visited;だから、余裕があればそこの実装もする
{
	WallSet back = pcLiDAR.identifyWall(LiDAR_degree::BACK);
	std::cout << ", back: " << (int)back.left << " " << (int)back.center << " " << (int)back.right;
	if ((back.left == WallType::type0 || back.left == WallType::typeNo) && back.center == WallType::center_n && (back.right == WallType::type0 || back.right == WallType::typeNo)) {
		if (back_tile == TileState::UNKNOWN) { // 未探索
			PDoT.back = canGo::GO;
		}
		else if (back_tile == TileState::HOLE || back_tile == TileState::AREA3to4 || back_tile == TileState::AREA1to4) {
			PDoT.back = canGo::NO;
		}
		else { // 何かしらのタイルを探索済み
			PDoT.back = canGo::VISITED;
		}
	}
	else {
		PDoT.back = canGo::NO;
	}
	back_mp = back;
}

void searchLeft(PotentialDirectionsOfTravel& PDoT, WallSet& left_mp, const TileState& left_tile)
{
	WallSet left = pcLiDAR.identifyWall(LiDAR_degree::LEFT);
	std::cout << ", left: " << (int)left.left << " " << (int)left.center << " " << (int)left.right;
	if ((left.left == WallType::type0 || left.left == WallType::typeNo) && left.center == WallType::center_n && (left.right == WallType::type0 || left.right == WallType::typeNo)) {
		if (left_tile == TileState::UNKNOWN) { // 未探索
			PDoT.left = canGo::GO;
		}
		else if (left_tile == TileState::HOLE || left_tile == TileState::AREA3to4 || left_tile == TileState::AREA1to4) {
			PDoT.left = canGo::NO;
		}
		else { // 何かしらのタイルを探索済み
			PDoT.left = canGo::VISITED;
		}
	}
	else {
		PDoT.left = canGo::NO;
	}
	left_mp = left;
}

void searchRight(PotentialDirectionsOfTravel& PDoT, WallSet& right_mp, const TileState& right_tile)
{
	WallSet right = pcLiDAR.identifyWall(LiDAR_degree::RIGHT);
	std::cout << ", right: " << (int)right.left << " " << (int)right.center << " " << (int)right.right << endl;
	if ((right.left == WallType::type0 || right.left == WallType::typeNo) && right.center == WallType::center_n && (right.right == WallType::type0 || right.right == WallType::typeNo)) {
		if (right_tile == TileState::UNKNOWN) { // 未探索
			PDoT.right = canGo::GO;
		}
		else if (right_tile == TileState::HOLE || right_tile == TileState::AREA3to4 || right_tile == TileState::AREA1to4) {
			PDoT.right = canGo::NO;
		}
		else { // 何かしらのタイルを探索済み
			PDoT.right = canGo::VISITED;
		}
	}
	else {
		PDoT.right = canGo::NO;
	}
	right_mp = right;
}

void HoleIsThere(const double& angle)
{
	if (abs(angle - 90) < 5) {
		mapper.markTileAs({ mapper.currentTile_R.x + 1,mapper.currentTile_R.z }, TileState::HOLE);
	}
	else if (abs(angle - 180) < 5) {
		mapper.markTileAs({ mapper.currentTile_R.x,mapper.currentTile_R.z - 1 }, TileState::HOLE);
	}
	else if (abs(angle - 270) < 5) {
		mapper.markTileAs({ mapper.currentTile_R.x - 1,mapper.currentTile_R.z }, TileState::HOLE);
	}
	else if ((angle <= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
		mapper.markTileAs({ mapper.currentTile_R.x,mapper.currentTile_R.z + 1 }, TileState::HOLE);
	}
	tank.gpsTrace(gps.last_expectedPos,4);
	gps.returnTolastPos();
}

void Area4IsThere(const double& angle) // チェックポイントタイルだけ踏んでやるHAHAHA
{
	if (abs(angle - 90) < 5) {
		searchAround(angle);
		tank.gpsTrace(gps.moveTiles(1, 0), 4);
		tank.gpsTrace(gps.moveTiles(-2, 0), 4);
		double angleN = gyro.getGyro();
		mapper.updatePostion(angleN);
	}
	else if (abs(angle - 180) < 5) {
		searchAround(angle);
		tank.gpsTrace(gps.moveTiles(0, -1), 4);
		tank.gpsTrace(gps.moveTiles(0, 2), 4);
		double angleN = gyro.getGyro();
		mapper.updatePostion(angleN);
	}
	else if (abs(angle - 270) < 5) {
		searchAround(angle);
		tank.gpsTrace(gps.moveTiles(-1, 0), 4);
		tank.gpsTrace(gps.moveTiles(2, 0), 4);
		double angleN = gyro.getGyro();
		mapper.updatePostion(angleN);
	}
	else if ((angle <= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
		searchAround(angle);
		tank.gpsTrace(gps.moveTiles(0, 1), 4);
		tank.gpsTrace(gps.moveTiles(0, -2), 4);
		double angleN = gyro.getGyro();
		mapper.updatePostion(angleN);
	}
}