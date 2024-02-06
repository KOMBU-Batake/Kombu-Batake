#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <math.h>
#include <vector>
#include <algorithm>
#include <map>
#include <unordered_set>

#include "../../lib/GlobalPositioningSystem/GlobalPositioningSystem.h"
#include "../../lib/easyLiDAR/easyLiDAR.h"
#include "../../lib/IMU/IMU.h"
#include "../../lib/PointCloudLiDAR/PointCloudLiDAR.h"
#include "../../lib/ColorSensor/ColorSensor.h"
#include "./Map.h"

/* ��o�p�̃}�b�v�����ADFS�ɂ����p�����
 *
 * ���W�̕ϐ����̖�����_A�����Ă�����̂�ABSOLUTE�܂��΍��W��
 * _R�����Ă�����̂�RELATIVE�܂葊�΍��W��\����
 *
 * �����ł̐�΍��W��map_A�ł̃^�C���̍��W�A���΍��W�̓X�^�[�g�^�C���ɑ΂�����W������
 * �Ȃ̂Ő�΍��W�͕��̒l����肦�Ȃ����A���΍��W�͕��̒l����邱�Ƃ�����
 *
 * ���X�g���ł�x,z�͊�{�I�ɐ�ΓI�Ȓl�������g��
 */

using namespace webots;
using namespace std;

extern Robot* robot;
extern GlobalPositioningSystem gps;

extern int timeStep;

class Map2 {
public:
	// �������Ɠ������炢�厖����
	vector<vector<string>> map_A = vector<vector<string>>(5, vector<string>(5, "-")); // �Ƃ肠����1x1�̃}�b�v�����
	// �������Ɠ������炢�厖����

	void addNorth(const int i = 1); // �k�����Ƀ^�C����ǉ�
	void addSouth(const int i = 1); // ������Ƀ^�C����ǉ�
	void addWest(const int j = 1); // �������Ƀ^�C����ǉ�
	void addEast(const int j = 1); // �������Ƀ^�C����ǉ�
private:

	MapAddress left_top_R = { 0, 0 }, right_bottom_R = { 0, 0 };
};