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

using namespace webots;
using namespace std;

// Robotインスタンスの作成
Robot* robot = new Robot();

// タイムステップの取得
int timeStep = (int)robot->getBasicTimeStep();

/* もぉたぁ */
Motor* leftMotor = robot->getMotor("leftWheel motor");
Motor* rightMotor = robot->getMotor("rightWheel motor");

/* ぽじしょんせんさぁ っていうか えんこぉだぁ */
PositionSensor* leftEncoder = leftMotor->getPositionSensor();
PositionSensor* rightEncoder = rightMotor->getPositionSensor();

/* らいだぁ */
Lidar* lidar = robot->getLidar("lidar");

/* でぃすたんすせんさぁ */
DistanceSensor* leftToF = robot->getDistanceSensor("leftToF");
DistanceSensor* rightToF = robot->getDistanceSensor("rightToF");

/* からぁせんさぁ */
Camera* colorCam = robot->getCamera("ColorSensor");

/* きゃめら */
Camera* leftCam = robot->getCamera("leftCamera");
Camera* rightCam = robot->getCamera("rightCamera");

/* じーぴーえす */
GPS* gps = robot->getGPS("gps");

/* あいえむゆぅ */
InertialUnit* IMU = robot->getInertialUnit("IMU");

ColorSensor colorsensor(colorCam);
GyroZ gyro;

void enableDevices() {
	leftEncoder->enable(timeStep);
	rightEncoder->enable(timeStep);
	lidar->enable(timeStep);
	leftToF->enable(timeStep);
	rightToF->enable(timeStep);
	colorCam->enable(timeStep);
	leftCam->enable(timeStep);
	rightCam->enable(timeStep);
	gps->enable(timeStep);
	IMU->enable(timeStep);
}