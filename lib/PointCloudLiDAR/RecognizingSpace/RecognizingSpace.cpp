#include "RecognizingSpace.h"

// 6cmずつ動かす
RoadAccess RecognizingSpace(vector<XZcoordinate>& poitns)
{
	RoadAccess roadAccess;
	bool notWall = true;
	// 前方向
	XZcoordinate centerPoint = { 0, 0 };
	while (notWall) {
		centerPoint = { centerPoint.x, centerPoint.z+6 };
		for (auto & point : poitns) {
			if ((pow(centerPoint.x - point.x, 2) + pow(centerPoint.z - point.z, 2)) < 12.25) {
				notWall = false;
				break;
			}
		}
	}
	roadAccess.front = (int8_t)(centerPoint.z-6);
	
	// 後方向
	notWall = true;
	centerPoint = { 0, 0 };
	while (notWall) {
		centerPoint = { centerPoint.x, centerPoint.z - 6 };
		for (auto & point : poitns) {
			if ((pow(centerPoint.x - point.x, 2) + pow(centerPoint.z - point.z, 2)) < 12.25) {
				notWall = false;
				break;
			}
		}
	}
	roadAccess.back = (int8_t)abs(centerPoint.z+6);

	// 右方向
	notWall = true;
	centerPoint = { 0, 0 };
	while (notWall) {
		centerPoint = { centerPoint.x + 6, centerPoint.z };
		for (auto& point : poitns) {
			if (pow(centerPoint.x - point.x, 2) + pow(centerPoint.z - point.z, 2) < 12.25) {
				notWall = false;
				break;
			}
		}
	}
	roadAccess.right = (int8_t)(centerPoint.x-6);

	// 左方向
	notWall = true;
	centerPoint = { 0, 0 };
	while (notWall) {
		centerPoint = { centerPoint.x - 6, centerPoint.z };
		for (auto& point : poitns) {
			if (pow(centerPoint.x - point.x, 2) + pow(centerPoint.z - point.z, 2) < 12.25) {
				notWall = false;
				break;
			}
		}
	}
	roadAccess.left = (int8_t)abs(centerPoint.x+6);
	return roadAccess;
}
