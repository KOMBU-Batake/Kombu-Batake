#include "DFS.h"

void DFS() {
	vector<MapAddress> stack = {mapper.currentTile_R};
	int tail = -1;
	while (robot->step(timeStep) != -1) {
		// ���݂̃^�C���̏����擾�A�L�^����
		colorsensor.update();
		TileState col = colorsensor.getTileColor();
		mapper.markTileAs(mapper.currentTile_R, col);
		//// �i�s�����̑I��
		double angle = gyro.getGyro();
		LiDAR_degree direction_of_travel = searchAround(angle);
		mapper.printMap();
	}
}

LiDAR_degree searchAround(double angle) {
	PotentialDirectionsOfTravel PDoT = { WallState::WALL,WallState::WALL,WallState::WALL,WallState::WALL };
	lidar.updateLiDAR(); // LiDAR�̒l���X�V

	WallState front_mp,back_mp,left_mp,right_mp, front_li, back_li, left_li, right_li;
	mapper.getAroundWallState(mapper.currentTile_R, front_mp, back_mp, left_mp, right_mp, angle); // �}�b�v��ł̎��͂̕ǂ̗L�����擾
	TileState front_tile, back_tile, left_tile, right_tile;
	mapper.getAroundTileState(mapper.currentTile_R, front_tile, back_tile, left_tile, right_tile, angle); // �}�b�v��ł̎��͂̃^�C���̗L�����擾
	
	searchFront(PDoT, front_mp, front_li, front_tile, angle); // �O
	searchBack(PDoT, back_mp, back_li, back_tile, angle); // ���
	searchLeft(PDoT, left_mp, left_li, left_tile, angle); // ��
	searchRight(PDoT, right_mp, right_li, right_tile, angle); // �E

	// �ǂ��}�b�v�ɓo�^
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

	// �i�s�����̑I��
	if (PDoT.left == WallState::noWALL) {

	}
	return LiDAR_degree::FRONT;
}

void searchFront(PotentialDirectionsOfTravel& PDoT, WallState& front_mp, WallState& front_li, TileState& front_tile, double& angle)
{
	if (front_mp == WallState::noWALL) {
		if (front_tile == TileState::UNKNOWN) { // ���T��
			PDoT.front = WallState::noWALL;
		}
		else if (front_tile == TileState::SWAMP || front_tile == TileState::AREA3to4 || front_tile == TileState::AREA1to4) { // ���܂��̓G���A3to4,1to4�͒ʂ�Ȃ�
			PDoT.front = WallState::WALL;
		}
		else { // ��������̃^�C����T���ς�
			PDoT.front = WallState::visited;
		}
	}
	else if (front_mp == WallState::unknown) { // lidar
		front_li = lidar.isWall(LiDAR_degree::FRONT, (float)angle);
		if (front_li == WallState::noWALL) PDoT.front = WallState::noWALL;
		front_mp = front_li;
	}
	else { // ��������̕�
		PDoT.front = WallState::WALL;
	}
}

void searchBack(PotentialDirectionsOfTravel& PDoT, WallState& back_mp, WallState& back_li, TileState& back_tile, double& angle) // ToDo: ��{�I��WallState::visited;������A�]�T������΂����̎���������
{
if (back_mp == WallState::noWALL) {
		if (back_tile == TileState::UNKNOWN) { // ���T��
			PDoT.back = WallState::noWALL;
		}
		else if (back_tile == TileState::SWAMP || back_tile == TileState::AREA3to4 || back_tile == TileState::AREA1to4) {
			PDoT.back = WallState::WALL;
		}
		else { // ��������̃^�C����T���ς�
			PDoT.back = WallState::visited;
		}
	}
	else if (back_mp == WallState::unknown) { // lidar
		back_li = lidar.isWall(LiDAR_degree::BACK, (float)angle);
		if (back_li == WallState::noWALL) PDoT.back = WallState::noWALL;
		back_mp = back_li;
	}
	else { // ��������̕�
		PDoT.back = WallState::WALL;
	}
}

void searchLeft(PotentialDirectionsOfTravel& PDoT, WallState& left_mp, WallState& left_li, TileState& left_tile, double& angle)
{
	if (left_mp == WallState::noWALL) {
		if (left_tile == TileState::UNKNOWN) { // ���T��
			PDoT.left = WallState::noWALL;
		}
		else if (left_tile == TileState::SWAMP || left_tile == TileState::AREA3to4 || left_tile == TileState::AREA1to4) {
			PDoT.left = WallState::WALL;
		}
		else { // ��������̃^�C����T���ς�
			PDoT.left = WallState::visited;
		}
	}
	else if (left_mp == WallState::unknown) { // lidar
		left_li = lidar.isWall(LiDAR_degree::LEFT, (float)angle);
		if (left_li == WallState::noWALL) PDoT.left = WallState::noWALL;
		left_mp = left_li;
	}
	else { // ��������̕�
		PDoT.left = WallState::WALL;
	}
}

void searchRight(PotentialDirectionsOfTravel& PDoT, WallState& right_mp, WallState& right_li, TileState& right_tile, double& angle)
{
	if (right_mp == WallState::noWALL) {
		if (right_tile == TileState::UNKNOWN) { // ���T��
			PDoT.right = WallState::noWALL;
		}
		else if (right_tile == TileState::SWAMP || right_tile == TileState::AREA3to4 || right_tile == TileState::AREA1to4) {
			PDoT.right = WallState::WALL;
		}
		else { // ��������̃^�C����T���ς�
			PDoT.right = WallState::visited;
		}
	}
	else if (right_mp == WallState::unknown) { // lidar
		right_li = lidar.isWall(LiDAR_degree::RIGHT, (float)angle);
		if (right_li == WallState::noWALL) PDoT.right = WallState::noWALL;
		right_mp = right_li;
	}
	else { // ��������̕�
		PDoT.right = WallState::WALL;
	}
}