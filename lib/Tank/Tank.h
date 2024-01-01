#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

using namespace webots;
using namespace std;

enum class StopMode {
	BRAKE,
	HOLD,
};

extern Robot* robot;
extern int timeStep;

class Tank {
public:
	Tank(Motor* _leftMotor, Motor* _rightMotor, PositionSensor* _leftEncoder, PositionSensor* _righhtEncoder);
	
	double getLeftEncoder() { // ‘f‚Ì•¨‚Æ”ä‚×‚Änan‚Ìˆ—‚ª’Ç‰Á‚³‚ê‚Ä‚¢‚é 
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
	
	void setPosition(double left, double right, double leftSpeed, double rightSpeed, bool ifcout = false, bool ChangeSpeed = true);

	void setVelocity(double leftSpeed, double rightSpeed) {
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
};