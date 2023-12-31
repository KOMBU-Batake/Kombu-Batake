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

#include "ColorSensor.h"
#include "IMU.h"
#include "GlobalPositioningSystem.h"
#include "ToF.h"

/* �f�o�C�X�̐ݒ�A�ȉ���������� */

using namespace webots;
using namespace std;

// Robot�C���X�^���X�̍쐬
Robot* robot = new Robot();

// �^�C���X�e�b�v�̎擾
int timeStep = (int)robot->getBasicTimeStep();

/* �������� */
Motor* leftMotor = robot->getMotor("leftWheel motor");
Motor* rightMotor = robot->getMotor("rightWheel motor");

/* �ۂ�����񂹂񂳂� ���Ă����� ���񂱂����� */
PositionSensor* leftEncoder = leftMotor->getPositionSensor();
PositionSensor* rightEncoder = rightMotor->getPositionSensor();

/* �炢���� */
Lidar* lidar = robot->getLidar("lidar");

/* �ł������񂷂��񂳂� */
DistanceSensor* leftToFSensor = robot->getDistanceSensor("leftToF");
DistanceSensor* rightToFSensor = robot->getDistanceSensor("rightToF");

/* ���炟���񂳂� */
Camera* colorCam = robot->getCamera("ColorSensor");

/* ����߂� */
Camera* leftCam = robot->getCamera("leftCamera");
Camera* rightCam = robot->getCamera("rightCamera");

/* ���[�ҁ[���� */
GPS* gpsXZ = robot->getGPS("gps");

/* �������ނ䂣 */
InertialUnit* IMU = robot->getInertialUnit("IMU");

ColorSensor colorsensor(colorCam);
GyroZ gyro;
GlobalPositioningSystem gps;
ToFSensor leftToF(leftToFSensor), rightToF(rightToFSensor);

void enableDevices() {
	leftEncoder->enable(timeStep);
	rightEncoder->enable(timeStep);
	lidar->enable(timeStep);
	leftToFSensor->enable(timeStep);
	rightToFSensor->enable(timeStep);
	colorCam->enable(timeStep);
	leftCam->enable(timeStep);
	rightCam->enable(timeStep);
	gpsXZ->enable(timeStep);
	IMU->enable(timeStep);
}