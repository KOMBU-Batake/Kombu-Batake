#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Lidar.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>

#include "../lib/ColorSensor/ColorSensor.h"
#include "../lib/IMU/IMU.h"
#include "../lib/GlobalPositioningSystem/GlobalPositioningSystem.h"
#include "../lib/ToF/ToF.h"
#include "../lib/Tank/Tank.h"
#include "../lib/easyLiDAR/easyLiDAR.h"
#include "../lib/PointCloudLiDAR/PointCloudLiDAR.h"
//#include "../lib/myMath/myMath.h"
#include "../src/Map/Map.h"
#include "../src/DFS/DFS.h"

/* �f�o�C�X�̐ݒ�A�ȉ���������� */

using namespace webots;
using namespace std;

// Robot�C���X�^���X�̍쐬
Robot* robot = new Robot();

// �^�C���X�e�b�v�̎擾
int timeStep = (int)robot->getBasicTimeStep(); // =16

/* �������� */
Motor* leftMotor = robot->getMotor("leftWheel motor");
Motor* rightMotor = robot->getMotor("rightWheel motor");

/* �ۂ�����񂹂񂳂� ���Ă����� ���񂱂����� */
PositionSensor* leftEncoder = leftMotor->getPositionSensor();
PositionSensor* rightEncoder = rightMotor->getPositionSensor();

/* �炢���� */
Lidar* centralLidar = robot->getLidar("lidar");

/* �ł������񂷂��񂳂� */
DistanceSensor* leftToFSensor = robot->getDistanceSensor("leftToF");
DistanceSensor* rightToFSensor = robot->getDistanceSensor("rightToF");

/* ���炟���񂳂� */
Camera* colorCam = robot->getCamera("ColorSensor");

/* ����߂� */
//Camera* leftCam = robot->getCamera("leftCamera");
//Camera* rightCam = robot->getCamera("rightCamera");

/* ���[�ҁ[���� */
GPS* gpsXZ = robot->getGPS("gps");

/* �������ނ䂣 */
InertialUnit* IMU = robot->getInertialUnit("IMU");

Emitter* emitter = robot->getEmitter("emitter");
Receiver* receiver = robot->getReceiver("receiver");

int16_t pcModelBox::counter = -1;
ColorSensor colorsensor(colorCam);
GyroZ gyro;
GlobalPositioningSystem gps;
ToFSensor leftToF(leftToFSensor), rightToF(rightToFSensor);
Tank tank(leftMotor, rightMotor, leftEncoder, rightEncoder);
LiDAR lidar;
Map mapper;
PointCloudLiDAR pcLiDAR;
//MyMath myMath;

void enableDevices() {
	leftEncoder->enable(timeStep);
	rightEncoder->enable(timeStep);
	centralLidar->enable(timeStep);
	leftToFSensor->enable(timeStep);
	rightToFSensor->enable(timeStep);
	colorCam->enable(timeStep);
	//leftCam->enable(timeStep);
	//rightCam->enable(timeStep);
	IMU->enable(timeStep);
	gpsXZ->enable(timeStep);
	receiver->enable(timeStep);
	robot->step(timeStep); // delay 16ms
	gps.recoedStartPosition();
}