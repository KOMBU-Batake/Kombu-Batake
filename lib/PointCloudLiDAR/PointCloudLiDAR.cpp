#include "PointCloudLiDAR.h"

pcModelBox::pcModelBox(const RangeLR tagrange, const vector<float> _model) {
	model = _model;
	range = tagrange;
	ManagementNumber = ++counter;
	if (range.left == -1) {
		left = right = center = estimatedWalls::gomi; // åüçıèúÇØ
		wallSet = { WallType::gomi,WallType::gomi,WallType::gomi };
	}
	else {
		int tmp = ManagementNumber % 11;
		switch (tmp)
		{
		case 0:
			right = estimatedWalls::type12;
			wallSet.right = WallType::type0;
			break;
		case 1:
			right = estimatedWalls::type9;
			wallSet.right = WallType::type1;
			break;
		case 2:
			right = estimatedWalls::type9;
			wallSet.right = WallType::type2;
			break;
		case 3:
			right = estimatedWalls::type6;
			wallSet.right = WallType::type3;
			break;
		case 4:
			right = estimatedWalls::type12;
			wallSet.right = WallType::type4;
			break;
		case 5:
			right = estimatedWalls::type6;
			wallSet.right = WallType::type5;
			break;
		case 6:
			right = estimatedWalls::type3;
			wallSet.right = WallType::type6;
			break;
		case 7:
			right = estimatedWalls::type3;
			wallSet.right = WallType::type7;
			break;
		case 8:
			right = estimatedWalls::type0;
			wallSet.right = WallType::type8;
			break;
		case 9:
			right = estimatedWalls::type6;
			wallSet.right = WallType::type9;
			break;
		case 10:
			right = estimatedWalls::type0;
			wallSet.right = WallType::type10;
			break;
		default:
			break;
		}

		if (ManagementNumber >= 242) {
			center = estimatedWalls::frontWall;
			wallSet.center = WallType::center_s;
		}
		else if (ManagementNumber >= 121) {
			center = estimatedWalls::backWall;
			wallSet.center = WallType::center_o;
		}

		int tmp2 = (ManagementNumber / 11) % 11;
		switch (tmp2)
				{
		case 0:
				left = estimatedWalls::type12;
				wallSet.left = WallType::type0;
				break;
		case 1:
			left = estimatedWalls::type12;
				wallSet.left = WallType::type1;
				break;
		case 2:
			left = estimatedWalls::type6;
			wallSet.left = WallType::type2;
			break;
		case 3:
			left = estimatedWalls::type9;
			wallSet.left = WallType::type3;
			break;
		case 4:
			left = estimatedWalls::type9;
			wallSet.left = WallType::type4;
			break;
		case 5:
			left = estimatedWalls::type6;
			wallSet.left = WallType::type5;
			break;
		case 6:
			left = estimatedWalls::type6;
			wallSet.left = WallType::type6;
			break;
		case 7:
			left = estimatedWalls::type3;
			wallSet.left = WallType::type7;
			break;
		case 8:
			left = estimatedWalls::type3;
			wallSet.left = WallType::type8;
			break;
		case 9:
			left = estimatedWalls::type0;
			wallSet.left = WallType::type9;
			break;
		case 10:
			left = estimatedWalls::type0;
			wallSet.left = WallType::type10;
			break;
		default:
			break;
		}
	}
}

void PointCloudLiDAR::update(const float angle) {
	rangeImage = centralLidar->getRangeImage();
	converttoPointCloud();
	float gyro_angle = (float)gyro.getGyro();
	fixPointCloudAngle(angle, gyro_angle);
}

void PointCloudLiDAR::NarrowDownWalls(const int& leftCount, const int& rightCount, estimatedWalls& left, estimatedWalls& right) {
	if (leftCount < 25) {
		left = estimatedWalls::type12;
	}
	else if (leftCount < 30) {
		left = estimatedWalls::type9;
	}
	else if (leftCount <= 37) {
		left = estimatedWalls::type6;
	}
	else if (leftCount < 50) {
		left = estimatedWalls::type3;
	}
	else {
		left = estimatedWalls::type0;
	}

	if (rightCount < 25) {
		right = estimatedWalls::type12;
	}
	else if (rightCount < 30) {
		right = estimatedWalls::type9;
	}
	else if (rightCount <= 37) {
		right = estimatedWalls::type6;
	}
	else if (rightCount < 50) {
		right = estimatedWalls::type3;
	}
	else {
		right = estimatedWalls::type0;
	}
}

void PointCloudLiDAR::convertRELATIVEtoABSLOUTE(float& angle) { // ëäëŒäpÇê‚ëŒäpÇ…ïœä∑
	//double g_angle = 360 - gyro.getGyro();
	//angle = 360 - angle; // å¸Ç´ÇîΩì]
	//angle -= g_angle;
	angle = (float)gyro.getGyro() - angle;
	if (angle < 0) angle += 360;
}

void PointCloudLiDAR::convertRELATIVEtoABSLOUTE(float& angle, const float& gyro) { // ëäëŒäpÇê‚ëŒäpÇ…ïœä∑ IMUÇÃílÇéÛÇØéÊÇÈver
	angle = gyro - angle;
	if (angle < 0) angle += 360;
}

void PointCloudLiDAR::convertABSLOUTEtoRELATIVE(float& angle) { // ê‚ëŒäpÇëäëŒäpÇ…ïœä∑
	//double g_angle = 360 - gyro.getGyro();
	//angle += g_angle;
	angle += (float)gyro.getGyro();
	if (angle > 360) angle -= 360;
}

void PointCloudLiDAR::convertABSLOUTEtoRELATIVE(float& angle, const float& gyro) { // ê‚ëŒäpÇëäëŒäpÇ…ïœä∑ IMUÇÃílÇéÛÇØéÊÇÈver
	angle += gyro;
	if (angle > 360) angle -= 360;
}

void PointCloudLiDAR::modelSamplimg(recoedingMode mode) {
	tank.setDireciton(90, 3);
	move_update_display(gps.expectedPos, 1, mode);
	for (int i = 2; i < 12; i++) {
		GPSPosition goalPos = gps.moveTiles(1, 0);
		tank.gpsTrace(goalPos, 3);
		move_update_display(goalPos,i,mode);
	}
	tank.gpsTrace(gps.moveTiles(-10, 0), 3);
	tank.setDireciton(90, 3);
}

void PointCloudLiDAR::move_update_display(GPSPosition goalPos,int j, recoedingMode mode) {
	update(goalPos);
	vector<XZcoordinate> model_left = { pointCloud[384] }, model_right;
	float centralZ = pointCloud[384].z;
	// ç∂ï˚å¸ 
	int8_t count_left = 0;
	while (1) {
		count_left++;
		if ((centralZ - pointCloud[384 - count_left].z) <= 5) {
			model_left.push_back(pointCloud[384 - count_left]);
			rotate(model_left.rbegin(), model_left.rbegin() + 1, model_left.rend());
			//cout << "model left; " << pointCloud[384 - count_left].z << endl;
		}
		else {
			count_left--;
			break;
		}
	}
	// âEï˚å¸
	int8_t count_right = 0;
	while (1) {
		count_right++;
		if ((pointCloud[384 + count_right].z - centralZ) <= 5) {
			model_right.push_back(pointCloud[384 + count_right]);
			//cout << "model right; " << pointCloud[384 + count_right].z << endl;
		}
		else {
			count_right--;
			break;
		}
	}
	model_left.insert(model_left.end(), model_right.begin(), model_right.end());
	if (mode == recoedingMode::model) {
		printVectorModel(model_left, j, {count_left,count_right});
	}
	else if (mode == recoedingMode::excel_2d) {
		printVectorExcel2D(model_left, j);
	}
}

void PointCloudLiDAR::getRangeImage(vector<float>& examinee, const LiDAR_degree& direction) {
	if (direction == LiDAR_degree::FRONT) {
		for (int i = 476; i < 512; i++) {
			examinee[i - 476] = pointCloud[i].z;
		}
		for (int i = 0; i <= 36; i++) {
			examinee[i + 36] = pointCloud[i].z;
		}
	}
	else if (direction == LiDAR_degree::RIGHT) {
		for (int i = 92; i <= 164; i++) {
			examinee[i - 92] = pointCloud[i].x;
		}
	}
	else if (direction == LiDAR_degree::BACK) {
		for (int i = 220; i <= 292; i++) {
			examinee[i - 220] = -1 * pointCloud[i].z;
		}
	}
	else if (direction == LiDAR_degree::LEFT) {
		for (int i = 348; i <= 420; i++) {
			examinee[i - 348] = -1 * pointCloud[i].x;
		}
	}
}

void PointCloudLiDAR::displayAllfloatVector(vector<float>& vec) {
	cout << "size;" << vec.size() << ", ";
	for (int i = 0; i < vec.size(); i++) {
		cout << vec[i] << "F,";
	}
	cout << endl;
}

void PointCloudLiDAR::displayAllXZcoordinateVector(vector<XZcoordinate>& vec, int j) {
	cout << j << ": size:" << vec.size() << "; ";
	for (int i = 0; i < vec.size(); i++) {
		cout << vec[i].x << "F, ";
	}
	cout << endl;
}

void PointCloudLiDAR::printVectorExcel2D(vector<XZcoordinate>& vec, int j)
{
	for (int i = 0; i < vec.size(); i++) {
		cout << vec[i].x << ", " << vec[i].z << endl;;
	}
}

void PointCloudLiDAR::printVectorExcel1D(vector<float>& vec, int j)
{
	for (int i = 0; i < vec.size(); i++) {
		cout << vec[i] << endl;;
	}
}

void PointCloudLiDAR::printVectorModel(vector<XZcoordinate>& vec, int& j, RangeLR range)
{
	cout << "num; " << j << ";pcModelBox({ " << (int)range.left << ", " << (int)range.right << " }, " << "{ ";
	for (int i = 0; i < vec.size(); i++) {
		cout << abs(vec[i].x) << "F, ";
	}
	cout << "})," << endl;
}


void PointCloudLiDAR::printNum() {
	for (int i = 0; i < 363; i++) {
		//cout << "MN; " << models[i].ManagementNumber << ", left; " << (int)models[i].left << ", right; " << (int)models[i].right << ", center; " << (int)models[i].center << endl;
		cout << "MN; " << models[i].ManagementNumber << ", left; " << (int)models[i].wallSet.left << ", right; " << (int)models[i].wallSet.right << ", center; " << (int)models[i].wallSet.center << endl;
	}
}