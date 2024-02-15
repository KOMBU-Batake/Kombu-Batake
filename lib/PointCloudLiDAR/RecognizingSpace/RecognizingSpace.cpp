#include "RecognizingSpace.h"

// 6cmÇ∏Ç¬ìÆÇ©Ç∑
RoadAccess RecognizingSpaceSimple(vector<XZcoordinate>& poitns)
{
	RoadAccess roadAccess = {6,6,6,6};
	bool notWall = true;
	// ëOï˚å¸
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
	
	// å„ï˚å¸
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

	// âEï˚å¸
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

	// ç∂ï˚å¸
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

moveableArea RecognizingSpace(vector<XZcoordinate>& poitns)
{
	RoadAccess access = RecognizingSpaceSimple(poitns);
	int8_t frontMax = (int8_t)(access.front / 6);
	int8_t backMax = (int8_t)(access.back / 6);
	vector<LeftRightAccess> front(frontMax), back(backMax);
	front.push_back({ (int8_t)(access.left / 6), (int8_t)(access.right / 6)});
	for (int i = 0; i < frontMax; i++) {
		LeftRightAccess leftRight = LeftRightAccessSimple(poitns, (float)i * 6 + 6);
		cout << "front, " << i << " : " << (int)leftRight.left << ", " << (int)leftRight.right << endl;
		front.push_back( leftRight );
	}
	for (int i = 0; i < backMax; i++) {
		LeftRightAccess leftRight = LeftRightAccessSimple(poitns, -1 * (float)i * 6 - 6);
		cout << "back, " << i << " : " << (int)leftRight.left << ", " << (int)leftRight.right << endl;
		back.push_back( leftRight );
	}
	return { front, back, access };
}

LeftRightAccess LeftRightAccessSimple(vector<XZcoordinate>& poitns, float z)
{
	LeftRightAccess leftRight = { 0,0 };
	// ç∂ï˚å¸
	bool notWall = true;
	XZcoordinate centerPoint = { 0, z };
	while (notWall) {
		centerPoint = { centerPoint.x - 6, centerPoint.z };
		for (auto& point : poitns) {
			if (pow(centerPoint.x - point.x, 2) + pow(centerPoint.z - point.z, 2) < 12.25) {
				notWall = false;
				break;
			}
		}
	}
	leftRight.left = (int8_t)abs(centerPoint.x / 6 + 1);

	// âEï˚å¸
	notWall = true;
	centerPoint = { 0, z };
	while (notWall) {
		centerPoint = { centerPoint.x + 6, centerPoint.z };
		for (auto& point : poitns) {
			if (pow(centerPoint.x - point.x, 2) + pow(centerPoint.z - point.z, 2) < 12.25) {
				notWall = false;
				break;
			}
		}
	}
	leftRight.right = (int8_t)abs(centerPoint.x / 6 - 1);

	return leftRight;
}
