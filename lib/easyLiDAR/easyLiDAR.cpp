#include "easyLiDAR.h"

DetailsofWall LiDAR::isWall(LiDAR_degree direction, float degree) {

	if (direction == LiDAR_degree::RELATIVE || direction == LiDAR_degree::ABSOLUTE) { // そのまんま指定先の値で判断する。
		float distance = getDistance(direction, degree);
		if (distance > 6) return DetailsofWall::noWALL;
		else return DetailsofWall::WALL;
	}
	else {
		directionInfo disinfo = deg_options(direction, degree); // 全部相対角に変換

		vector<float> distance_list;
		vector<int> degree_list;
		
		int leftDeg512 = disinfo.leftDeg512;
		int rightDeg512 = disinfo.rightDeg512;
		int centerDeg512 = disinfo.centerDeg512;
		cout << "center; " << centerDeg512 << ", left: " << leftDeg512 << ", right: " << rightDeg512 << endl;

		// 範囲のデータを取得
		if (!disinfo.flag0to360) { // どちらのフラグも立っていない
			//cout << "no flag" << endl;
			distance_list.resize(rightDeg512 - leftDeg512 + 1); // left->rightの順に入っていく
			degree_list.resize(rightDeg512 - leftDeg512 + 1);
			for (int i = 0; i <= (rightDeg512 - leftDeg512); i++) {
				distance_list[i] = rangeImage[i + leftDeg512 + 1024] * 100;
				degree_list[i] = centerDeg512 - (i + leftDeg512);
				//cout << i + leftDeg512 << ", " << distance_list[i] << endl;
			}
		}
		else { // フラグが立った ToDo: 本当はdegree_listの値をdisindo.degreeに合わせなきゃいけないんだけどFRONT以外でここを踏むことはないはずだからほってる
			//cout << "flag" << endl;
			int dn = 511 - leftDeg512;
			distance_list.resize(dn + rightDeg512 + 2);
			degree_list.resize(dn + rightDeg512 + 2);
			for (int i = leftDeg512; i <= 511; i++) {
				distance_list[i - leftDeg512] = rangeImage[i + 1024] * 100;
				degree_list[i - leftDeg512] = i;
				//cout << i << ", " << distance_list[i-leftDeg512] << endl;
			}
			for (int i = 0; i <= rightDeg512; i++) {
				distance_list[i + dn + 1] = rangeImage[i + 1024] * 100;
				degree_list[i + dn + 1] = i;
				//cout << i << ", " << distance_list[i + dn + 1] << endl;
			}
		}
		//cout << "****************" << endl;
		if (direction == LiDAR_degree::FRONT_LEFT || direction == LiDAR_degree::FRONT_RIGHT) {
			for (int i = 0; i < distance_list.size(); i++) {
				//cout << abs(distance_list[i] * sin(pd_degrees_to_rad((centerDeg512 - degree_list[i]) * 45 / 64))) << ", " << disinfo.distance << ", " << ((centerDeg512 - degree_list[i]) * 45 / 64) << endl;
				if (abs(distance_list[i] * sin(pd_degrees_to_rad((centerDeg512 - degree_list[i]) * 45 / 64))) < disinfo.distance) {
					return DetailsofWall::maybeWALL; // 1つでも反応したら壁あり
				}
			}
			return DetailsofWall::maybeNOWALL;
		}
		else
		{
			vector<bool> wall_list(distance_list.size(), false); // falseで初期化
			for (int i = 0; i < distance_list.size(); i++) {
				if (abs(distance_list[i] * cos(pd_degrees_to_rad(degree_list[i] * 45 / 64))) < disinfo.distance) { // 各要素について壁かどうか判断
					wall_list[i] = true;
				}
				//cout << abs(distance_list[i] * cos(pd_degrees_to_rad(degree_list[i] * 45 / 64))) << ", " << disinfo.distance << endl;
			}

			if (all_of(wall_list.begin(), wall_list.end(), [](bool i) { return i; })) { // 全て壁
				return DetailsofWall::WALL;
			}
			else if (none_of(wall_list.begin(), wall_list.end(), [](bool i) { return i; })) { // 全て壁じゃない
				return DetailsofWall::noWALL;
			}
			else if (wall_list[0]) { // 左側に壁
				return DetailsofWall::leftWALL;
			}
			else if (wall_list[wall_list.size() - 1]) { // 右側に壁
				return DetailsofWall::rightWALL;
			}
			else { // 中央に壁 というか、それ以外の意
				return DetailsofWall::cneterWALL;
			}
		}
	}
}