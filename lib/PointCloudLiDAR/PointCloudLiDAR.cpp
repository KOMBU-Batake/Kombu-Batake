#include "PointCloudLiDAR.h"

void PointCloudLiDAR::move_update_display(int j) {
	cout << "==================================================" << endl;
	tank.gpsTrace(gps.moveTiles(1, 0), 3);
	//vector<float> model0(73);
	//vector<float> model_LS(59);
	//vector<float> model_SL(59);
	//vector<float> model_SS(45);
	vector<XZcoordinate> model0_XZ(73);
	vector<XZcoordinate> model_LS_XZ(59);
	vector<XZcoordinate> model_SL_XZ(59);
	vector<XZcoordinate> model_SS_XZ(45);
	update();
	for (int i = 348; i <= 420; i++) {
		//model0[i - 348] = rangeImage[i + 1024];
		model0_XZ[i - 348].x = pointCloud[i].x;
		model0_XZ[i - 348].z = pointCloud[i].z;
	}
	//copy(model0.begin(), model0.begin() + 59, model_LS.begin());
	//copy(model0.begin() + 14, model0.end(), model_SL.begin());
	//copy(model0.begin() + 14, model0.begin() + 59, model_SS.begin());
	copy(model0_XZ.begin(), model0_XZ.begin() + 59, model_LS_XZ.begin());
	copy(model0_XZ.begin() + 14, model0_XZ.end(), model_SL_XZ.begin());
	copy(model0_XZ.begin() + 14, model0_XZ.begin() + 59, model_SS_XZ.begin());
	//displayAllfloatVector(model0);
	//displayAllfloatVector(model_LS);
	//displayAllfloatVector(model_SL);
	//displayAllfloatVector(model_SS);
	displayAllXZcoordinateVector(model0_XZ, j);
	displayAllXZcoordinateVector(model_LS_XZ, j);
	displayAllXZcoordinateVector(model_SL_XZ, j);
	displayAllXZcoordinateVector(model_SS_XZ, j);
}

void PointCloudLiDAR::displayAllfloatVector(vector<float>& vec) {
	cout << "size;" << vec.size() << ", ";
	for (int i = 0; i < vec.size(); i++) {
		cout << vec[i] << ",";
	}
	cout << endl;
}

void PointCloudLiDAR::displayAllXZcoordinateVector(vector<XZcoordinate>& vec, int j) {
	cout << j << "; size;" << vec.size() << ", ";
	for (int i = 0; i < vec.size(); i++) {
		cout << vec[i].x << ", ";
	}
	cout << endl;
}