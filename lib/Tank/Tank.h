#pragma once

#include <iostream>
#include <math.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include "../GlobalPositioningSystem/GlobalPositioningSystem.h"
#include "../IMU/IMU.h"
#include "../ColorSensor/ColorSensor2.h"
#include "../MyCam/MyCam.h"

using namespace webots;
using namespace std;

enum class StopMode {
	BRAKE,
	HOLD,
	COAST,
};

enum class unit {
	rad,
	degrees,
	m,
	cm,
	mm,
};

enum class Direction_of_Travel {
	x,
	z,
	diagonal,

	/* ここから下はTank.cppでしか使わない 他所で使うんじゃねぇ */
	x_plus,
	x_minus,
	z_plus,
	z_minus,
};

extern Robot* robot;
extern int timeStep;
extern GlobalPositioningSystem gps;
extern GyroZ gyro;
extern ColorSensor2 colorsensor;
extern MyCam myCam;

class Tank {
public:
	Tank(Motor* _leftMotor, Motor* _rightMotor, PositionSensor* _leftEncoder, PositionSensor* _righhtEncoder);
	
	double getLeftEncoder() { // 素の物と比べてnanの処理が追加されている 
		double leftEncoderValue = this->leftEncoder->getValue();
		if (isnan(leftEncoderValue)) {
			return 0;
		} else return leftEncoderValue;
	}

	double getRightEncoder() {
		double rightEncoderValue = this->rightEncoder->getValue();
		if (isnan(rightEncoderValue)) {
			return 0;
		} else return rightEncoderValue;
	}
	
	/* 今思ったけど、コレ色々都合が悪いからあんまり使わない方がいいね */
	void setPosition(double left, double right, double leftSpeed, double rightSpeed, unit unit = unit::degrees, bool ifcout = false, bool ChangeSpeed = true);

	/* #論外 */
	void setVelocity(double leftSpeed, double rightSpeed) { // 引数はdouble
		if (abs(leftSpeed) > maxVelocity) { cout << "exceed max velocity on left motor" << endl; return; }
		if (abs(rightSpeed) > maxVelocity) { cout << "exceed max velocity on right motor" << endl; return; }
		if (leftSpeed > 0) {
			if (setV_goal_left != goalPosition::plusINFINITY) {
				leftMotor->setPosition(INFINITY);
				setV_goal_left = goalPosition::plusINFINITY;
			}
		}
		else if (setV_goal_left != goalPosition::minusINFINITY) {
			leftMotor->setPosition(-1*INFINITY);
			setV_goal_left = goalPosition::minusINFINITY;
		}

		if (rightSpeed > 0) {
			if (setV_goal_right != goalPosition::plusINFINITY) {
				rightMotor->setPosition(INFINITY);
				setV_goal_right = goalPosition::plusINFINITY;
			}
		}
		else if (setV_goal_right != goalPosition::minusINFINITY) {
			rightMotor->setPosition(-1*INFINITY);
			setV_goal_right = goalPosition::minusINFINITY;
		}

		leftMotor->setVelocity(leftSpeed);
		rightMotor->setVelocity(rightSpeed);
	}

	void stop(const StopMode mode);

	/* その場で回転して向きを合わせる */
	void setDireciton(double direction, double maxspeed /*最大速度*/, const unit unit = unit::degrees);

	/* Ｘ軸方向、Y軸方向にGPSトレースする 位置はあらかたあっている前提で過度な修正はできない。 */
	bool gpsTrace(const GPSPosition& goal, double speed /*rad/s固定*/, const StopMode stopmode = StopMode::HOLD, int timeout_ms = 1000, Direction_of_Travel direction = Direction_of_Travel::diagonal); // ロボットはすでに進行方向を向いていることを前提とする

	static double pd_cm_to_rad(double cm) { // cmをラジアンに変換
		return cm / 2.05; // rθ(cm)/r(cm) =θ(rad)
	}

	static double pd_degrees_to_rad(double degrees) {
		return degrees * 3.1415926535 / 180;
	}

	static double pd_cm_to_degrees(double cm) { // cmを角度に変換
		return pd_cm_to_rad(cm) * 180 / 3.141592; // θ*180/π
	}

	static double pd_rad_to_degrees(double rad) {
		return rad * 180 / 3.1415926535;
	}

	bool checkColor() {
		//colorsensor.update();
		//TileState tileColor = colorsensor.getTileColor();
		//if (tileColor == TileState::HOLE) {
		//	return false;
		//}
		return true;
	}
private:
	enum class goalPosition :uint8_t{
		plusINFINITY,
		minusINFINITY,
		OTHER,
	};

	Motor* leftMotor;
	Motor* rightMotor;
	PositionSensor* leftEncoder;
	PositionSensor* rightEncoder;
	goalPosition setV_goal_left = goalPosition::OTHER;
	goalPosition setV_goal_right = goalPosition::OTHER;

	double maxVelocity = 6.28;

	enum Direction_of_Rotation {
		plus, // 反時計回り
		minus, // 時計回り
	};
};