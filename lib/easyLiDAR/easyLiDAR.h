#pragma once

#include <iostream>
#include <vector>
#include <algorithm>
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include "../IMU/IMU.h"
#include "../Tank/Tank.h"

/* #easyなわけないだろ */

using namespace webots;
using namespace std;

extern Lidar* centralLidar;
extern GyroZ gyro;

enum class LiDAR_degree {
	RELATIVE, // 機体から見た相対的な角度
	ABSOLUTE, // フィールドから見た絶対的な角度(方位とも言える)

	LEFT, // 機体の左側
	RIGHT,
	FRONT,
	BACK,

	FRONT_RIGHT,
	FRONT_LEFT,
};

typedef struct {
	float degree; // 目標の角度
	float diff; // 前後の範囲
	float distance; // これより大きければ壁ナシと判断
}directionInfo;

enum class DetailsofWall {
	noWALL,     // 0
	WALL,       // 1
	leftWALL,   // 2
	rightWALL,  // 3
	cneterWALL, // 4
	maybeWALL,  // 5
	maybeNOWALL,// 6
};

class LiDAR {
public:
	/* 値の更新 */
	void updateLiDAR() {
		rangeImage = centralLidar->getRangeImage();
	}

	/* 上から3層目の値を返す */
	float getDistance(LiDAR_degree deg_option, float degree = 0) {
		if (deg_option != LiDAR_degree::RELATIVE) degree = deg_options(deg_option, degree).degree; // 諸々を相対角に変換
		// 角度を0~360から0~512にする は?
		degree = degree * 512 / 360;
		degree = round(degree);
		if (degree == 512) degree = 0; // 512番の値は存在しない(0と等しい)
		int num = (int)degree;
		return rangeImage[num + 1024] * 100;
	}

	DetailsofWall isWall(LiDAR_degree direction, float degree = 0) {

		if (direction == LiDAR_degree::RELATIVE || direction == LiDAR_degree::ABSOLUTE) { // そのまんま指定先の値で判断する。
			float distance = getDistance(direction,degree);
			if (distance > 6) return DetailsofWall::noWALL;
			else return DetailsofWall::WALL;
		}
		else {
			directionInfo disinfo = deg_options(direction, degree); // 全部相対角に変換

			bool over360 = false, less0 = false;
			vector<float> distance_list;
			vector<int> degree_list;
			float leftDeg  = disinfo.degree - disinfo.diff; 
			float rightDeg = disinfo.degree + disinfo.diff;
			if (leftDeg < 0) {
				less0 = true;
				leftDeg += 360;
			}
			if (rightDeg >= 360) { // この=には意味がある 360はconvert360to512にかけると0に変換されるからそのフラグを予め立てる
				over360 = true;
				rightDeg -= 360;
			}
			int leftDeg512 = convert360to512(leftDeg);
			int rightDeg512 = convert360to512(rightDeg);
			int centerDeg512 = convert360to512(disinfo.degree);
			cout << "center; " << centerDeg512 << ", left: " << leftDeg512 << ", right: " << rightDeg512 << endl;

			// 範囲のデータを取得
			if (!less0 && !over360) { // どちらのフラグも立っていない
				cout << "no flag" << endl;
				distance_list.resize(rightDeg512 - leftDeg512 + 1); // left->rightの順に入っていく
				degree_list.resize(rightDeg512 - leftDeg512 + 1);
				for (int i = 0; i <= (rightDeg512 - leftDeg512); i++) {
					distance_list[i] = rangeImage[i + leftDeg512 + 1024] * 100;
					degree_list[i] = centerDeg512 - (i + leftDeg512);
					cout << i + leftDeg512 << ", " << distance_list[i] << endl;
				}
			}
			else { // フラグが立った ToDo: 本当はdegree_listの値をdisindo.degreeに合わせなきゃいけないんだけどFRONT以外でここを踏むことはないはずだからほってる
				cout << "flag" << endl;
				int dn = 511 - leftDeg512;
				distance_list.resize(dn + rightDeg512 + 2);
				degree_list.resize(dn + rightDeg512 + 2);
				for (int i = leftDeg512; i <= 511; i++) {
					distance_list[i-leftDeg512] = rangeImage[i + 1024] * 100;
					degree_list[i - leftDeg512] = i;
					cout << i << ", " << distance_list[i-leftDeg512] << endl;
				}
				for (int i = 0; i <= rightDeg512; i++) {
					distance_list[i + dn + 1] = rangeImage[i + 1024] * 100;
					degree_list[i + dn + 1] = i;
					cout << i << ", " << distance_list[i + dn + 1] << endl;
				}
			}
			cout << "****************" << endl;
			if (direction == LiDAR_degree::FRONT_LEFT || direction == LiDAR_degree::FRONT_RIGHT) {
				for (int i = 0; i < distance_list.size(); i++) {
					cout << abs(distance_list[i] * sin(pd_degrees_to_rad((centerDeg512 - degree_list[i]) * 45 / 64))) << ", " << disinfo.distance << ", " << ((centerDeg512 - degree_list[i]) * 45 / 64) << endl;
					if (abs( distance_list[i] * sin(pd_degrees_to_rad((centerDeg512 - degree_list[i]) * 45 / 64)) ) < disinfo.distance) {
						return DetailsofWall::maybeWALL; // 1つでも反応したら壁あり
					}
				}
				return DetailsofWall::maybeNOWALL;
			}
			else 
			{
				vector<bool> wall_list(distance_list.size(),false); // falseで初期化
				for (int i = 0; i < distance_list.size(); i++) {
					if (abs(distance_list[i] * cos(pd_degrees_to_rad(degree_list[i] * 45 / 64))) < disinfo.distance) { // 各要素について壁かどうか判断
						wall_list[i] = true;
					}
					//cout << abs(distance_list[i] * cos(pd_degrees_to_rad(degree_list[i] * 45 / 64))) << ", " << disinfo.distance << endl;
				}

				if (all_of(wall_list.begin(), wall_list.end(), [](bool i){ return i; })) { // 全て壁
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

	uint8_t convertCMtoTILE(float& distanceCM) {
		return (uint8_t)round((distanceCM - 5)/12);
	}

private:
	const float* rangeImage = 0;

	directionInfo LEFT = { 270, 22.5, (float)6.7 };
	directionInfo RIGHT = { 90, 22.5, (float)6.7 };
	directionInfo BACK = { 180, 22.5, (float)6.7 };
	directionInfo FRONT = { 0, 22.5, (float)6.7 };
	directionInfo FRONT_LEFT = { (float)(270 + 63.435), 3.5, 6.5 };
	directionInfo FRONT_RIGHT = { (float)(90 - 63.435), 3.5, 6.5 };

	void convertABSLOUTEtoRELATIVE(float& angle) { // 絶対角を相対角に変換
		//double g_angle = 360 - gyro.getGyro();
		//angle = 360 - angle; // 向きを反転
		//angle -= g_angle;
		angle = (float)gyro.getGyro() - angle;
		if (angle < 0) angle += 360;
	}

	directionInfo deg_options(LiDAR_degree deg_option, float degree = 0) {
		if (deg_option == LiDAR_degree::ABSOLUTE) {
			convertABSLOUTEtoRELATIVE(degree);
			return { degree,0 };
		}
		else if (deg_option == LiDAR_degree::LEFT) {
			return LEFT;
		}
		else if (deg_option == LiDAR_degree::RIGHT) {
			return RIGHT;
		}
		else if (deg_option == LiDAR_degree::BACK) {
			return BACK;
		}
		else if (deg_option == LiDAR_degree::FRONT) {
			return FRONT;
		}
		else if (deg_option == LiDAR_degree::FRONT_LEFT) {
			return FRONT_LEFT;
		}
		else if (deg_option == LiDAR_degree::FRONT_RIGHT) {
			return FRONT_RIGHT;
		}
		return { -1,-1,-1 }; // 知るか
	}

	int convert360to512(float degree){
		degree = degree * 512 / 360;
		degree = round(degree);
		if (degree == 512) degree = 0; // 512番の値は存在しない(0と等しい)
		return (int)degree;
	}

	static double pd_degrees_to_rad(double degrees) {
		return degrees * 3.1415926535 / 180;
	}
};