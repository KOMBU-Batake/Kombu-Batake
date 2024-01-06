#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

#include "../GlobalPositioningSystem/GlobalPositioningSystem.h"
#include "../IMU/IMU.h"

using namespace webots;
using namespace std;

enum class StopMode {
	BRAKE,
	HOLD,
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
};

extern Robot* robot;
extern int timeStep;
extern GlobalPositioningSystem gps;
extern GyroZ gyro;

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
	
	void setPosition(double left, double right, double leftSpeed, double rightSpeed, unit unit = unit::degrees, bool ifcout = false, bool ChangeSpeed = true);

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
	void setDireciton(double direction, double speed, const unit unit = unit::degrees);

	/* 単にX軸方向、Y軸方向にGPSトレースする */
	void gpsTraveSimple(const GPSPosition& goal, const double& speed /*rad/s固定*/ , const Direction_of_Travel& direction ); // ロボットはすでに進行方向を向いていることを前提とする
private:
	enum class goalPosition :uint8_t{
		plusINFINITY,
		minusINFINITY,
		OTHER,
	};

	double pd_cm_to_rad(double cm) { // cmをラジアンに変換
		return cm / 2.05; // rθ(cm)/r(cm) =θ(rad)
	}

	double pd_degrees_to_rad(double degrees) {
		return degrees * 3.1415926535 / 180;
	}

	double pd_cm_to_degrees(double cm) { // cmを角度に変換
	  return pd_cm_to_rad(cm) * 180 / 3.141592; // θ*180/π
	}

	double pd_rad_to_degrees(double rad) {
		return rad * 180 / 3.1415926535;
	}

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