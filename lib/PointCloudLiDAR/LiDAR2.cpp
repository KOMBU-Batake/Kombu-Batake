#include "LiDAR2.h"

NcmPoints LiDAR2::getNcmPoints(const LiDAR_degree& direction, uint16_t range) {
	// 中央10cmのデータを取得
	int center = centralDegreeMap[direction];
	NcmPoints ncmP;
	int16_t half_range = range / 2;

	if (direction == LiDAR_degree::LEFT || direction == LiDAR_degree::RIGHT) {
		ncmP.model_left = { pointCloud[center] };
		float centralZ = pointCloud[center].z;
		// 右方向
		ncmP.count_right = 0;
		while (1) {
			ncmP.count_right++;
			if (abs(pointCloud[center + ncmP.count_right].z - centralZ) <= half_range) {
				ncmP.model_right.push_back(pointCloud[center + ncmP.count_right]);
			}
			else {
				ncmP.count_right--;
				break;
			}
		}
		// 左方向 
		ncmP.count_left = 0;
		while (1) {
			ncmP.count_left++;
			if (abs(centralZ - pointCloud[center - ncmP.count_left].z) <= half_range) {
				ncmP.model_left.push_back(pointCloud[center - ncmP.count_left]);
				rotate(ncmP.model_left.rbegin(), ncmP.model_left.rbegin() + 1, ncmP.model_left.rend());
			}
			else {
				ncmP.count_left--;
				break;
			}
		}
	}
	else
	{
		ncmP.model_left = { pointCloud[center] };
		float centralX = pointCloud[center].x;
		// 右方向
		ncmP.count_right = 0;
		while (1) {
			ncmP.count_right++;
			if (abs(pointCloud[center + ncmP.count_right].x - centralX) <= half_range) {
				ncmP.model_right.push_back(pointCloud[center + ncmP.count_right]);
			}
			else {
				ncmP.count_right--;
				break;
			}
		}
		// 左方向 
		if (direction == LiDAR_degree::FRONT) center = 512;
		ncmP.count_left = 0;
		while (1) {
			ncmP.count_left++;
			if (abs(centralX - pointCloud[center - ncmP.count_left].x) <= half_range) {
				ncmP.model_left.push_back(pointCloud[center - ncmP.count_left]);
				rotate(ncmP.model_left.rbegin(), ncmP.model_left.rbegin() + 1, ncmP.model_left.rend());
			}
			else {
				ncmP.count_left--;
				break;
			}
		}
	}
	return ncmP;
}