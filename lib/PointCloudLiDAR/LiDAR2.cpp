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

WallSet LiDAR2::getWallType(const LiDAR_degree& direction)
{
	// 中央10cmのデータを取得
	NcmPoints pointsSet = getNcmPoints(direction, 10);

	MAXandMIN max_min = getMAX_MIN(pointsSet, direction);
	
	return WallSet();
}

MAXandMIN LiDAR2::getMAX_MIN(NcmPoints& pointsSet, LiDAR_degree direction)
{
	// 都合のいいように座標を回転
	switch (direction) {
		case LiDAR_degree::BACK:
			transform(pointsSet.model_left.begin(), pointsSet.model_left.end(), pointsSet.model_left.begin(),
				[](XZcoordinate element) { 
					element.z *= -1;
					element.x *= -1;
					return element; 
				});
			transform(pointsSet.model_right.begin(), pointsSet.model_right.end(), pointsSet.model_right.begin(),
				[](XZcoordinate element) { 
					element.z *= -1; 
					element.x *= -1;
					return element; 
				});
			break;
		case LiDAR_degree::LEFT:
			transform(pointsSet.model_left.begin(), pointsSet.model_left.end(), pointsSet.model_left.begin(),
				[](XZcoordinate element, float tmp) { 
					tmp = element.x;
					element.x = -1 * element.z;
					element.z = tmp;
					return element; 
				});
			transform(pointsSet.model_right.begin(), pointsSet.model_right.end(), pointsSet.model_right.begin(),
				[](XZcoordinate element, float tmp) {
					tmp = element.x;
					element.x = -1 * element.z;
					element.z = tmp;
					return element; 
				});
			break;
		case LiDAR_degree::RIGHT:
			transform(pointsSet.model_left.begin(), pointsSet.model_left.end(), pointsSet.model_left.begin(),
				[](XZcoordinate element, float tmp) {
					tmp = element.x;
					element.x =element.z;
					element.z = -1 * tmp;
					return element;
				});
			transform(pointsSet.model_right.begin(), pointsSet.model_right.end(), pointsSet.model_right.begin(),
				[](XZcoordinate element, float tmp) {
					tmp = element.x;
					element.x = element.z;
					element.z = -1 * tmp;
					return element;
				});
			break;
		default:
			break;
	}

	// 最大値と最小値を取得
	XZcoordinate maxLeft = *max_element(pointsSet.model_left.begin(), pointsSet.model_left.end(),
		[](XZcoordinate a, XZcoordinate b) { return a.z < b.z; });
	XZcoordinate minLeft = *min_element(pointsSet.model_left.begin(), pointsSet.model_left.end(),
		[](XZcoordinate a, XZcoordinate b) { return a.z < b.z; });
	XZcoordinate maxRight = *max_element(pointsSet.model_right.begin(), pointsSet.model_right.end(),
		[](XZcoordinate a, XZcoordinate b) { return a.z < b.z; });
	XZcoordinate minRight = *min_element(pointsSet.model_right.begin(), pointsSet.model_right.end(),
		[](XZcoordinate a, XZcoordinate b) { return a.z < b.z; });
	return MAXandMIN{ maxLeft.z, minLeft.z, maxRight.z, minRight.z };
}
