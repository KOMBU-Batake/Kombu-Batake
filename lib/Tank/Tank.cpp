#include "Tank.h"

static void setGoalPosition(const double& leftPos, const double& rightPos, double& leftSp, double& rightSp, const double& leftStartPos, const double& rightStartPos, double& leftGoalPos, double& rightGoalPos);

Tank::Tank(Motor* _leftMotor, Motor* _rightMotor, PositionSensor* _leftEncoder, PositionSensor* _righhtEncoder) {
	this->leftMotor = _leftMotor;
	this->rightMotor = _rightMotor;
	this->leftEncoder = _leftEncoder;
	this->rightEncoder = _righhtEncoder;
}

void Tank::setPosition(double left, double right, double leftSpeed, double rightSpeed, bool ifcout, bool ChangeSpeed) {
	setV_goal_left = goalPosition::OTHER;
	setV_goal_right = goalPosition::OTHER;
	double leftStartPosition = leftEncoder->getValue();
	double rightStartPosition = rightEncoder->getValue();
	// nanの処理
	if (isnan(leftStartPosition) || isnan(rightStartPosition)) {
		leftStartPosition = 0; rightStartPosition = 0;
	}
	double leftGoal = leftStartPosition;
	double rightGoal = rightStartPosition;
	setGoalPosition(left, right, leftSpeed, rightSpeed, leftStartPosition, rightStartPosition, leftGoal, rightGoal);
	leftMotor->setPosition(leftGoal);
	rightMotor->setPosition(rightGoal);
	if (ChangeSpeed) {
		leftMotor->setVelocity(leftSpeed);
		rightMotor->setVelocity(rightSpeed);
	}
	cout << "leftSpeed: " << leftSpeed << ", rightSpeed: " << rightSpeed << endl;
	cout << "leftGoal: " << leftGoal << ", rightGoal: " << rightGoal << endl;
	while (1) {
		double presentLeftPosition = getLeftEncoder();
		double presentRightPosition = getRightEncoder();
		if (ifcout) {
			cout << "left position: " << presentLeftPosition << "; right position: " << presentRightPosition << endl;
		}
		if (abs(presentLeftPosition - leftGoal) < 0.01 || abs(presentRightPosition - rightGoal) < 0.01 || robot->step(timeStep) == -1) { // 0.01ぐらいが妥当でしょ
			break;
		}
	}
	robot->step(160);
	stop(StopMode::BRAKE);
}

static void setGoalPosition(const double& leftPos, const double& rightPos,
														double& leftSp, double& rightSp, 
														const double& leftStartPos, const double& rightStartPos, 
														double& leftGoalPos, double& rightGoalPos) {
	if ((leftPos > 0 && leftSp > 0) || (leftPos < 0 && leftSp < 0)) { // 両方正または両方負の場合は前進
		leftGoalPos = (abs(leftPos) * 3.141592 / 180) + leftStartPos;
		leftSp = abs(leftSp);
	}
	else if (leftPos > 0 && leftSp < 0) { // 片方正片方負の場合は後退
		leftGoalPos = (-1 * leftPos * 3.141592 / 180) + leftStartPos;
	}
	else if (leftPos < 0 && leftSp > 0) { // 上に同じ
		leftGoalPos = (leftPos * 3.141592 / 180) + leftStartPos;
		//leftSp = -1 * leftSp;
	}

	if ((rightPos > 0 && rightSp > 0) || (rightPos < 0 && rightSp < 0)) { // 両方正または両方負の場合は前進
		rightGoalPos = (abs(rightPos) * 3.141592 / 180) + rightStartPos;
		rightSp = abs(rightSp);
	}
	else if (rightPos > 0 && rightSp < 0) { // 片方正片方負の場合は後退
		rightGoalPos = (-1 * rightPos * 3.141592 / 180) + rightStartPos;
 	}
 	else if (rightPos < 0 && rightSp > 0) { // 上に同じ
		rightGoalPos = (rightPos * 3.141592 / 180) + rightStartPos;
		//rightSp = -1 * rightSp;
	}
}

void Tank::stop(const StopMode mode) {
	switch (mode) {
	case StopMode::BRAKE:
		leftMotor->setVelocity(0);
		rightMotor->setVelocity(0);
		break;
	case StopMode::HOLD:
		setPosition(getLeftEncoder(), getRightEncoder(), 0, 0, false, false);
		break;
	}
}