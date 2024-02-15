#pragma once

/*
 * 「シミュレーションの中でシミュレーション()してみよう」そんな気の狂った挑戦。
 * 深夜テンションでなきゃ抹殺しちゃうね。
 * 
 * 具体的にはLiDARの点をプロットしたXZ平面で直径7cmの円をLiDARの点に当たるまで動かす。
 * 
 */

#include <iostream>
#include <webots/Robot.hpp>
#include <math.h>
#include <vector>
#include <algorithm>
#include <map>
#include <unordered_set>

using namespace webots;
using namespace std;

extern Robot* robot;
extern int timeStep;

struct XZcoordinate {
	float x;
	float z;
};

struct RoadAccess {
	int8_t front;
	int8_t left;
	int8_t right;
	int8_t back;
};

struct LeftRightAccess {
	int8_t left;
	int8_t right;
};

struct moveableArea {
	RoadAccess access;
	moveableArea(vector<LeftRightAccess> front, vector<LeftRightAccess> back, RoadAccess access): access(access){	
		
	}
};

// 前後左右方向に進むことができる距離
// この値を基にマップを拡張してマッピングボーナスを少しでも確実にこそげ取るぱわー
RoadAccess RecognizingSpaceSimple(vector<XZcoordinate>& poitns);

moveableArea RecognizingSpace(vector<XZcoordinate>& poitns);

static LeftRightAccess LeftRightAccessSimple(vector<XZcoordinate>& poitns, float z);