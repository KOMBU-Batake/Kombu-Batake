#include "DFS.h"

void DFS() {
	vector<stackDFS> stack;
	vector<MapAddress> footprints = {mapper.currentTile_R};
	int tail = -1;
	colorsensor.update();
	obstacleState obstacle = colorsensor.obstacle();

	//mapper.printMap();
	cout << "DFS" << endl;
	int count_DFS = 0;
	while (robot->step(timeStep) != -1) {
		count_DFS++;
		cout << "count DFS: " << count_DFS << endl;
		// �p�x(����)���擾
		double angle = gyro.getGyro();

		// �ǂ̏����擾
		lidar2.update(gps.expectedPos);
		//for (int i = 0; i < lidar2.pointCloud.size(); i++) {
		//	cout << lidar2.pointCloud[i].x << ", " << lidar2.pointCloud[i].z << endl;;
		//}
		WallSet front_mp = lidar2.getWallType(LiDAR_degree::FRONT);
		WallSet back_mp = lidar2.getWallType(LiDAR_degree::BACK);
		WallSet left_mp = lidar2.getWallType(LiDAR_degree::LEFT);
		WallSet right_mp = lidar2.getWallType(LiDAR_degree::RIGHT);
		cout << "front: " << (int)front_mp.left << " " << (int)front_mp.center << " " << (int)front_mp.right << " / ";
		cout << "back: " << (int)back_mp.left << " " << (int)back_mp.center << " " << (int)back_mp.right << " / ";
		cout << "left: " << (int)left_mp.left << " " << (int)left_mp.center << " " << (int)left_mp.right << " / ";
		cout << "right: " << (int)right_mp.left << " " << (int)right_mp.center << " " << (int)right_mp.right << endl;

		// �ǂ��o�p�}�b�v�ɓo�^
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
		
		// ���ʃJ�����ŗ��Ƃ����̊m�F
		myCam.update();
		vector<bool> left_hole = myCam.leftHole();
		vector<bool> right_hole = myCam.rightHole();

		// �i�߂������T��
		canGoSet cango_set = lidar2.identifyCanGo();
		// �΂߂̑I�������ᖡ
		if ((int)cango_set.front < 3 || (int)cango_set.left < 3) cango_set.front_left = canGo::NO;
		if ((int)cango_set.front < 3 || (int)cango_set.right < 3) cango_set.front_right = canGo::NO;

		// ���Ƃ����̍l��
		if (obstacle.leftHoleState || obstacle.rightHoleState) cango_set.front = canGo::NO;
		if (left_hole[0] || left_hole[1]) cango_set.left = canGo::NO;
		if (right_hole[0] || right_hole[1]) cango_set.right = canGo::NO;

		// �������s���Ă������̂�
		reallyAbleToGo(cango_set.front, front_mp);
		reallyAbleToGo(cango_set.left, left_mp);
		reallyAbleToGo(cango_set.right, right_mp);
		reallyAbleToGo(cango_set.back, back_mp);

		// �T���p�}�b�v�ɋL�^ ���ɂ͏������܂Ȃ�(���Ƃ����̊m�F���ł��Ȃ�����)
		mapperS.markAroundStatus({ (int)cango_set.front < 3,
															 (int)cango_set.left < 3,
															 (int)cango_set.right < 3,
															 (int)cango_set.back < 3,
															 (int)cango_set.front_left < 3,
															 (int)cango_set.front_right < 3 }, 
															 angle, count_DFS == 1);
		mapperS.printMap();

		// �^�C���̏����擾
		vector<TileState> front_tile(2), back_tile(2), left_tile(2), right_tile(2), front_left_tile(2), front_right_tile(2);
		front_tile = mapper.getTileStateLR(mapper.currentTile_R, angle, LiDAR_degree::FRONT);
		back_tile = mapper.getTileStateLR(mapper.currentTile_R, angle, LiDAR_degree::BACK);
		left_tile = mapper.getTileStateLR(mapper.currentTile_R, angle, LiDAR_degree::LEFT);
		right_tile = mapper.getTileStateLR(mapper.currentTile_R, angle, LiDAR_degree::RIGHT);
		front_left_tile = mapper.getTileStateLR(mapper.currentTile_R, angle, LiDAR_degree::FRONT_LEFT);
		front_right_tile = mapper.getTileStateLR(mapper.currentTile_R, angle, LiDAR_degree::FRONT_RIGHT);

		// �i�s�����̑I��
		canGo canGoFront = judgeCanGo(front_tile, (int)cango_set.front < 3);
		cout << "front tile:" << (int)front_tile[0] << " " << (int)front_tile[1] << endl;
		canGo canGoBack = judgeCanGo(back_tile, (int)cango_set.back < 3);
		canGo canGoLeft = judgeCanGo(left_tile, (int)cango_set.left < 3);
		canGo canGoRight = judgeCanGo(right_tile, (int)cango_set.right < 3);
		canGo canGoFrontLeft = judgeCanGo(front_left_tile, (int)cango_set.front_left < 3);
		canGo canGoFrontRight = judgeCanGo(front_right_tile, (int)cango_set.front_right < 3);

		cout << "front: " << (int)canGoFront << " left: " << (int)canGoLeft << " right: " << (int)canGoRight << " back: " << (int)canGoBack << " front_left: " << (int)canGoFrontLeft << " front_right: " << (int)canGoFrontRight << endl;
		
		bool hasStack = addToStackDFS(stack, angle, canGoFront, canGoLeft, canGoRight, canGoBack, canGoFrontLeft, canGoFrontRight);
		if (stack.size() == 0) {
			break;
		}
		MapAddress nextTile = pickSatckDFS(stack);
		cout << "nextTile: " << nextTile.x << " " << nextTile.z << endl;
		if (nextTile.x == 123456789) { // �X�^�b�N����ɂȂ�����I��
			break;
		}

		// �i��
		vector<MapAddress> route = mapperS.getRoute(nextTile, footprints);
		bool isHole = false;

		for (auto& tile : route) {
			if (tile == *route.begin()) continue;
			int dx, dz;
			dx = tile.x - mapper.currentTile_R.x;
			dz = tile.z - mapper.currentTile_R.z;
			isHole = tank.gpsTrace(gps.moveTiles(dx, dz), 3, StopMode::COAST);
			angle = gyro.getGyro();
			cout << "angle: " << angle << endl;
			if (isHole) {
				mapper.updatePostion(angle);
				footprints.push_back(mapper.currentTile_R);
			}
		}
		if (!isHole) {
			HoleIsThere(angle);
		}
		cout << "======================" << endl;
		// �΂߂ɂ���Ă����ꍇ�͏C��
		if (!(abs(angle - 90) < 5 ||
			abs(angle - 180) < 5 ||
			abs(angle - 270) < 5 ||
			(angle >= 0 && angle < 5) || (angle > 355 && angle <= 360))) {
			// �C��
			cout << "fixing angle" << endl;
			vector<float> tmp = { 0,90,180,270,360 };
			auto min = min_element(tmp.begin(), tmp.end(), [&angle](float a, float b) { return abs(a - angle) < abs(b - angle); });
			tank.setDireciton(*min, 4);
			angle = gyro.getGyro();
		}
		colorsensor.update();
		TileState left_col = colorsensor.getLeftColor();
		colorsensor.getRightColor();
		obstacle = colorsensor.obstacle();
		TileState recordedTile = mapper.getTileState(mapper.currentTile_R);
		if (recordedTile != TileState::START){
			mapper.markTileAs(mapper.currentTile_R, left_col, angle);
			// ������X�^�b�N�̔r��
			removeFromStackDFS(stack, mapper.currentTile_R);
		}
		TileState wholeTile = mapper.getCurrentTile(mapper.currentTile_R);
		if (wholeTile != TileState::WALL) { // �S�ē����ł͂Ȃ�
			if (wholeTile == TileState::AREA1to4 || wholeTile == TileState::AREA3to4) {
				if (hasStack) { // �X�^�b�N��ǉ����Ă����Ȃ�����B
					stack.pop_back();
				}
				Area4IsThere(angle);
				angle = gyro.getGyro();
			}
		}

		deleteVisited(stack);
		if (stack.size() == 0) {
			break;
		}

		//angle = gyro.getGyro();
		//if (!isHole) {
		//	cout << "Hole is there" << endl;
		//	HoleIsThere(angle);
		//}
		//else {
		//	mapper.updatePostion(angle);
		//	// ���݂̃^�C���̏����擾�A�L�^����
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
		//		// �X�^�b�N�Ɍ��݂̃^�C����ǉ�
		//		if (!dontStack) stack.push_back(mapper.currentTile_R);
		//	}
		//}
	}
	vector<MapAddress> route = mapperS.getRoute(mapper.startTile_R, footprints);
	bool isHole = false;
	for (auto& tile : route) {
		if (tile == *route.begin()) continue;
		int dx, dz;
		dx = tile.x - mapper.currentTile_R.x;
		dz = tile.z - mapper.currentTile_R.z;
		isHole = tank.gpsTrace(gps.moveTiles(dx, dz), 3, StopMode::COAST);
		double angle = gyro.getGyro();
		if (isHole) {
			break;
		}
	}

	cout << "final submit " << endl;
	cout << "current tile: " << mapper.currentTile_R.x << " " << mapper.currentTile_R.z << endl;
	double angle = gyro.getGyro();
	mapper.updatePostion(angle);
	route = mapperS.getRoute(mapper.startTile_R, footprints);
	for (auto& tile : route) {
		if (tile == *route.begin()) continue;
		int dx, dz;
		dx = tile.x - mapper.currentTile_R.x;
		dz = tile.z - mapper.currentTile_R.z;
		isHole = tank.gpsTrace(gps.moveTiles(dx, dz), 3, StopMode::COAST);
		angle = gyro.getGyro();
		if (isHole) {
			mapper.updatePostion(angle);
			footprints.push_back(mapper.currentTile_R);
		}
	}
	mapper.replaceLineTo0();
	mapper.printMap();
	sendMap(mapper.map_A);

	char msg = 'M'; // Send map evaluate request
	emitter->send(&msg, sizeof(msg));

	while (robot->step(timeStep) != -1) {
		msg = 'E'; // Send an Exit message to get Map Bonus
		emitter->send(&msg, sizeof(msg));
		tank.setVelocity(-1, 1);
	}
}

MapAddress pickSatckDFS(vector<stackDFS>& stack)
{
	stackDFS top = stack.back();
	MapAddress next = {123456789,123456789 };
	// �΂�
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
		if (stack.size() == 0) {
			cout << "stack is empty" << endl;
			return next;
		}
		return pickSatckDFS(stack); // �Ȃ�������ċN
	}
	return next;
}

canGo judgeCanGo(const vector<TileState>& tileState, const bool canGo)
{
	if (canGo) {
		if (find(tileState.begin(), tileState.end(), TileState::UNKNOWN) != tileState.end()) { // ���T��
			if (all_of(tileState.begin(), tileState.end(), [](const TileState& state) { return state == TileState::UNKNOWN; })) {
				return canGo::GO;
			}
			return canGo::PARTLY_VISITED;
		}
		else if (any_of(tileState.begin(), tileState.end(), [](const TileState& state) { return state == TileState::HOLE || state == TileState::AREA3to4 || state == TileState::AREA1to4; })) { // �����G���A4������ {
			return canGo::NO;
		}
		else { // ��������̃^�C����T���ς�
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
	//cout << "front: " << (int)front << " left: " << (int)left << " right: " << (int)right << " back: " << (int)back << " front_left: " << (int)front_left << " front_right: " << (int)front_right << endl;
	//cout << "north: " << (int)north << " east: " << (int)east << " south: " << (int)south << " west: " << (int)west << " north_east: " << (int)north_east << " north_west: " << (int)north_west << " south_east: " << (int)south_east << " south_west: " << (int)south_west << endl;

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

void removeFromStackDFS(vector<stackDFS>& stack, const MapAddress& address)
{
	for (auto& stackElement : stack) {
		for (auto& go : stackElement.Go) {
			if (go == address) {
				stackElement.Go.erase(remove(stackElement.Go.begin(), stackElement.Go.end(), address), stackElement.Go.end());
			}
		}
		for (auto& partlyVisited : stackElement.PartlyVisited) {
			if (partlyVisited == address) {
				stackElement.PartlyVisited.erase(remove(stackElement.PartlyVisited.begin(), stackElement.PartlyVisited.end(), address), stackElement.PartlyVisited.end());
			}
		}
		for (auto& diagonal : stackElement.diagonal) {
			if (diagonal == address) {
				stackElement.diagonal.erase(remove(stackElement.diagonal.begin(), stackElement.diagonal.end(), address), stackElement.diagonal.end());
			}
		}
	}

}

void reallyAbleToGo(canGo& cango, const WallSet& wallset)
{
	if (cango == canGo::GO_leftO) {
		if (wallset.right == WallType::type8) {
			cango = canGo::NO;
		}
	} else if (cango == canGo::GO_rightO) {
		if (wallset.left == WallType::type7) {
			cango = canGo::NO;
		}
	}
}

void deleteVisited(vector<stackDFS>& stacks)
{
	if (stacks.size() == 0) return;
	for (auto stack = stacks.begin(); stack != stacks.end(); ++stack) {
		if (stack->Go.size() != 0) {
			for (auto addr = stack->Go.begin(); addr != stack->Go.end(); ++addr) {
				if (mapper.isAllVisited(*addr)) {
					stack->Go.erase(addr);
					--addr;
				}
			}
		}
		if (stack->PartlyVisited.size() != 0) {
			for (auto addr = stack->PartlyVisited.begin(); addr != stack->PartlyVisited.end(); ++addr) {
				if (mapper.isAllVisited(*addr)) {
					stack->PartlyVisited.erase(addr);
					--addr;
				}
			}
		}
		if (stack->Go.size() == 0 && stack->PartlyVisited.size() == 0) {
			// delete
			stack = stacks.erase(stack);
			--stack;
		}
	}
}

/* ��O�ɔ��}�X�i�߂邩�ۂ�
   ���a�͗]�T��������8cm�Ŋ��肷�� */
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

void Area4IsThere(const double& angle) // �`�F�b�N�|�C���g�^�C����������ł��HAHAHA
{
	if (abs(angle - 90) < 5) {
		tank.gpsTrace(gps.moveTiles(-2, 0), 5);
		double angleN = gyro.getGyro();
		mapper.updatePostion(angleN);
		mapper.updatePostion(angleN);
	}
	else if (abs(angle - 180) < 5) {
		tank.gpsTrace(gps.moveTiles(0, 2), 5);
		double angleN = gyro.getGyro();
		mapper.updatePostion(angleN);
		mapper.updatePostion(angleN);
	}
	else if (abs(angle - 270) < 5) {
		tank.gpsTrace(gps.moveTiles(2, 0), 5);
		double angleN = gyro.getGyro();
		mapper.updatePostion(angleN);
		mapper.updatePostion(angleN);
	}
	else if ((angle >= 0 && angle < 5) || (angle > 355 && angle <= 360)) {
		tank.gpsTrace(gps.moveTiles(0, -2), 5);
		double angleN = gyro.getGyro();
		mapper.updatePostion(angleN);
		mapper.updatePostion(angleN);
	}
}

void sendMap(vector<vector<string>>& map) { // malloc���g���Ă�̂��ǂ��Ȃ���
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
}