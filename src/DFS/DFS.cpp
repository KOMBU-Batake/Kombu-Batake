#include "DFS.h"

void DFS() {
	vector<MapAddress> stack = {mapper.currentTile_R};
	int tail = -1;
	while (robot->step(timeStep) != -1) {
		// 現在のタイルの情報を取得、記録する
		colorsensor.update();
		TileState col = colorsensor.getTileColor();
		mapper.markTileAs(mapper.currentTile_R, col);
		//// 進行方向の選択
		double angle = gyro.getGyro();
		LiDAR_degree direction_of_travel = searchAround(angle);
		mapper.printMap();
	}
}

LiDAR_degree searchAround(double angle) {
	PotentialDirectionsOfTravel PDoT = { WallState::WALL,WallState::WALL,WallState::WALL,WallState::WALL };
	lidar.updateLiDAR(); // LiDARの値を更新

	WallState front_mp,back_mp,left_mp,right_mp, front_li, back_li, left_li, right_li;
	mapper.getAroundWallState(mapper.currentTile_R, front_mp, back_mp, left_mp, right_mp, angle); // マップ上での周囲の壁の有無を取得
	TileState front_tile, back_tile, left_tile, right_tile;
	mapper.getAroundTileState(mapper.currentTile_R, front_tile, back_tile, left_tile, right_tile, angle); // マップ上での周囲のタイルの有無を取得
	
	searchFront(PDoT, front_mp, front_li, front_tile, angle); // 前
	searchBack(PDoT, back_mp, back_li, back_tile, angle); // 後ろ
	searchLeft(PDoT, left_mp, left_li, left_tile, angle); // 左
	searchRight(PDoT, right_mp, right_li, right_tile, angle); // 右

	// 壁をマップに登録
	if (abs(angle - 90) < 5) {
		mapper.markAroundWall(left_mp,right_mp,back_mp,front_mp);
	}
	else if (abs(angle - 180) < 5) {
		mapper.markAroundWall(front_mp,back_mp,left_mp,right_mp);
	}
	else if (abs(angle - 270) < 5) {
		mapper.markAroundWall(right_mp,left_mp,front_mp,back_mp);
	}
	else if ((angle <= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
		mapper.markAroundWall(back_mp,front_mp,right_mp,left_mp);
	}
	cout << "front: " << (int)PDoT.front << " back: " << (int)PDoT.back << " left: " << (int)PDoT.left << " right: " << (int)PDoT.right << endl;

	// 進行方向の選択
	if (PDoT.left == WallState::noWALL) {

	}
	return LiDAR_degree::FRONT;
}

void searchFront(PotentialDirectionsOfTravel& PDoT, WallState& front_mp, WallState& front_li, TileState& front_tile, double& angle)
{
	if (front_mp == WallState::noWALL) {
		if (front_tile == TileState::UNKNOWN) { // 未探索
			PDoT.front = WallState::noWALL;
		}
		else if (front_tile == TileState::SWAMP || front_tile == TileState::AREA3to4 || front_tile == TileState::AREA1to4) { // 沼またはエリア3to4,1to4は通らない
			PDoT.front = WallState::WALL;
		}
		else { // 何かしらのタイルを探索済み
			PDoT.front = WallState::visited;
		}
	}
	else if (front_mp == WallState::unknown) { // lidar
		front_li = lidar.isWall(LiDAR_degree::FRONT, (float)angle);
		if (front_li == WallState::noWALL) PDoT.front = WallState::noWALL;
		front_mp = front_li;
	}
	else { // 何かしらの壁
		PDoT.front = WallState::WALL;
	}
}

void searchBack(PotentialDirectionsOfTravel& PDoT, WallState& back_mp, WallState& back_li, TileState& back_tile, double& angle) // ToDo: 基本的にWallState::visited;だから、余裕があればそこの実装もする
{
if (back_mp == WallState::noWALL) {
		if (back_tile == TileState::UNKNOWN) { // 未探索
			PDoT.back = WallState::noWALL;
		}
		else if (back_tile == TileState::SWAMP || back_tile == TileState::AREA3to4 || back_tile == TileState::AREA1to4) {
			PDoT.back = WallState::WALL;
		}
		else { // 何かしらのタイルを探索済み
			PDoT.back = WallState::visited;
		}
	}
	else if (back_mp == WallState::unknown) { // lidar
		back_li = lidar.isWall(LiDAR_degree::BACK, (float)angle);
		if (back_li == WallState::noWALL) PDoT.back = WallState::noWALL;
		back_mp = back_li;
	}
	else { // 何かしらの壁
		PDoT.back = WallState::WALL;
	}
}

void searchLeft(PotentialDirectionsOfTravel& PDoT, WallState& left_mp, WallState& left_li, TileState& left_tile, double& angle)
{
	if (left_mp == WallState::noWALL) {
		if (left_tile == TileState::UNKNOWN) { // 未探索
			PDoT.left = WallState::noWALL;
		}
		else if (left_tile == TileState::SWAMP || left_tile == TileState::AREA3to4 || left_tile == TileState::AREA1to4) {
			PDoT.left = WallState::WALL;
		}
		else { // 何かしらのタイルを探索済み
			PDoT.left = WallState::visited;
		}
	}
	else if (left_mp == WallState::unknown) { // lidar
		left_li = lidar.isWall(LiDAR_degree::LEFT, (float)angle);
		if (left_li == WallState::noWALL) PDoT.left = WallState::noWALL;
		left_mp = left_li;
	}
	else { // 何かしらの壁
		PDoT.left = WallState::WALL;
	}
}

void searchRight(PotentialDirectionsOfTravel& PDoT, WallState& right_mp, WallState& right_li, TileState& right_tile, double& angle)
{
	if (right_mp == WallState::noWALL) {
		if (right_tile == TileState::UNKNOWN) { // 未探索
			PDoT.right = WallState::noWALL;
		}
		else if (right_tile == TileState::SWAMP || right_tile == TileState::AREA3to4 || right_tile == TileState::AREA1to4) {
			PDoT.right = WallState::WALL;
		}
		else { // 何かしらのタイルを探索済み
			PDoT.right = WallState::visited;
		}
	}
	else if (right_mp == WallState::unknown) { // lidar
		right_li = lidar.isWall(LiDAR_degree::RIGHT, (float)angle);
		if (right_li == WallState::noWALL) PDoT.right = WallState::noWALL;
		right_mp = right_li;
	}
	else { // 何かしらの壁
		PDoT.right = WallState::WALL;
	}
}