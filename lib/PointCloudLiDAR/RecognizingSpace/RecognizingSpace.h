#pragma once

/*
 * �u�V�~�����[�V�����̒��ŃV�~�����[�V����()���Ă݂悤�v����ȋC�̋���������B
 * �[��e���V�����łȂ��ᖕ�E�����Ⴄ�ˁB
 * 
 * ��̓I�ɂ�LiDAR�̓_���v���b�g����XZ���ʂŒ��a7cm�̉~��LiDAR�̓_�ɓ�����܂œ������B
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

// �O�㍶�E�����ɐi�ނ��Ƃ��ł��鋗��
// ���̒l����Ƀ}�b�v���g�����ă}�b�s���O�{�[�i�X�������ł��m���ɂ��������ς�[
RoadAccess RecognizingSpaceSimple(vector<XZcoordinate>& poitns);

moveableArea RecognizingSpace(vector<XZcoordinate>& poitns);

static LeftRightAccess LeftRightAccessSimple(vector<XZcoordinate>& poitns, float z);