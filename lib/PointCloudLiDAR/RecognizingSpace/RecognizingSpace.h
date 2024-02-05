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

typedef struct {
	float x;
	float z;
}XZcoordinate;

typedef struct {
	int8_t front;
	int8_t left;
	int8_t right;
	int8_t back;
}RoadAccess;

// �O�㍶�E�����ɐi�ނ��Ƃ��ł��鋗��
// ���̒l����Ƀ}�b�v���g�����ă}�b�s���O�{�[�i�X�������ł��m���ɂ��������
RoadAccess RecognizingSpace(vector<XZcoordinate>& poitns);