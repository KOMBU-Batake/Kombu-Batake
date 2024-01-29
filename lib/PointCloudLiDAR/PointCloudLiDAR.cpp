#include "PointCloudLiDAR.h"

void PointCloudLiDAR::modelSamplimg(recoedingMode mode) {
	tank.setDireciton(90, 3);
	move_update_display(gps.getPosition(), 1, mode);
	for (int i = 2; i < 12; i++) {
		GPSPosition goalPos = gps.moveTiles(1, 0);
		tank.gpsTrace(goalPos, 3);
		move_update_display(goalPos,i,mode);
	}
	tank.gpsTrace(gps.moveTiles(-10, 0), 3);
	tank.setDireciton(90, 3);
}

void PointCloudLiDAR::move_update_display(GPSPosition goalPos,int j, recoedingMode mode) {
	cout << "==================================================" << endl;
	//vector<float> model0(73);
	//vector<float> model_LS(59);
	//vector<float> model_SL(59);
	//vector<float> model_SS(45);
	//vector<XZcoordinate> model0_XZ(73);
	//vector<XZcoordinate> model_LS_XZ(59);
	//vector<XZcoordinate> model_SL_XZ(59);
	//vector<XZcoordinate> model_SS_XZ(45);

	update(goalPos);
	vector<XZcoordinate> model_left = { pointCloud[384] }, model_right;
	float centralZ = pointCloud[384].z;
	// ç∂ï˚å¸ 
	int count_left = 0;
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
	int count_right = 0;
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
	cout << "num: " << j << endl;
	cout << "model size:" << model_left.size() << endl;
	cout << "model_left.size():" << count_left << ", model_right.size():" << count_right << endl;
	if (mode == recoedingMode::model) {
		printVectorModel(model_left, j, {count_left,count_right});
	}
	else if (mode == recoedingMode::excel_2d) {
		printVectorExcel2D(model_left, j);
	}
	//for (int i = 348; i <= 420; i++) {
	//	//model0[i - 348] = rangeImage[i + 1024];
	//	model0_XZ[i - 348].x = pointCloud[i].x;
	//	model0_XZ[i - 348].z = pointCloud[i].z;
	//}
	//copy(model0.begin(), model0.begin() + 59, model_LS.begin());
	//copy(model0.begin() + 14, model0.end(), model_SL.begin());
	//copy(model0.begin() + 14, model0.begin() + 59, model_SS.begin());
	//copy(model0_XZ.begin(), model0_XZ.begin() + 59, model_LS_XZ.begin());
	//copy(model0_XZ.begin() + 14, model0_XZ.end(), model_SL_XZ.begin());
	//copy(model0_XZ.begin() + 14, model0_XZ.begin() + 59, model_SS_XZ.begin());
	//displayAllfloatVector(model0);
	//displayAllfloatVector(model_LS);
	//displayAllfloatVector(model_SL);
	//displayAllfloatVector(model_SS);
	//displayAllXZcoordinateVector(model0_XZ, j);
	//displayAllXZcoordinateVector(model_LS_XZ, j);
	//displayAllXZcoordinateVector(model_SL_XZ, j);
	//displayAllXZcoordinateVector(model_SS_XZ, j);
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

void PointCloudLiDAR::printVectorModel(vector<XZcoordinate>& vec, int& j, RangeLR range)
{
	cout << "{ " << range.left << ", " << range.right << " }, " << "{ ";
	for (int i = 0; i < vec.size(); i++) {
		cout << vec[i].x << "F, ";
	}
	cout << "}" << endl;
}
