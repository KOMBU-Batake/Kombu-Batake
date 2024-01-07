#include "Tank.h"

static void setGoalPosition(const double& leftPos, const double& rightPos, double& leftSp, double& rightSp, const double& leftStartPos, const double& rightStartPos, double& leftGoalPos, double& rightGoalPos);
static double pd_angle(double angle);

Tank::Tank(Motor* _leftMotor, Motor* _rightMotor, PositionSensor* _leftEncoder, PositionSensor* _righhtEncoder) {
	this->leftMotor = _leftMotor;
	this->rightMotor = _rightMotor;
	this->leftEncoder = _leftEncoder;
	this->rightEncoder = _righhtEncoder;
}

void Tank::setPosition(double left, double right, double leftSpeed, double rightSpeed, unit unit, bool ifcout, bool ChangeSpeed) {
	setV_goal_left = goalPosition::OTHER;
	setV_goal_right = goalPosition::OTHER;
	/* �P�ʂ̕␳ */
	if (unit == unit::cm) {
		left = pd_cm_to_rad(left);
		right = pd_cm_to_rad(right);
	}
	else if (unit == unit::degrees) {
		left = pd_degrees_to_rad(left);
		right = pd_degrees_to_rad(right);
	}
	else if (unit != unit::rad) {
		return; // �s�K�؂ȒP��
	}
	double leftStartPosition = leftEncoder->getValue();
	double rightStartPosition = rightEncoder->getValue();
	// nan�̏���
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
	if (ifcout) {
		cout << "leftSpeed: " << leftSpeed << ", rightSpeed: " << rightSpeed << endl;
		cout << "leftGoal: " << leftGoal << ", rightGoal: " << rightGoal << endl;
	}
	while (1) {
		double presentLeftPosition = getLeftEncoder();
		double presentRightPosition = getRightEncoder();
		if (ifcout) {
			cout << "left position: " << presentLeftPosition << "; right position: " << presentRightPosition << endl;
		}
		if (abs(presentLeftPosition - leftGoal) < 0.01 || abs(presentRightPosition - rightGoal) < 0.01 || robot->step(timeStep) == -1) { // 0.01���炢���Ó��ł���
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
	if ((leftPos > 0 && leftSp > 0) || (leftPos < 0 && leftSp < 0)) { // �������܂��͗������̏ꍇ�͑O�i
		leftGoalPos = abs(leftPos) + leftStartPos;
		leftSp = abs(leftSp);
	}
	else if (leftPos > 0 && leftSp < 0) { // �Е����Е����̏ꍇ�͌��
		leftGoalPos = (-1 * leftPos) + leftStartPos;
	}
	else if (leftPos < 0 && leftSp > 0) { // ��ɓ���
		leftGoalPos = leftPos + leftStartPos;
	}

	if ((rightPos > 0 && rightSp > 0) || (rightPos < 0 && rightSp < 0)) { // �������܂��͗������̏ꍇ�͑O�i
		rightGoalPos = abs(rightPos) + rightStartPos;
		rightSp = abs(rightSp);
	}
	else if (rightPos > 0 && rightSp < 0) { // �Е����Е����̏ꍇ�͌��
		rightGoalPos = (-1 * rightPos) + rightStartPos;
 	}
 	else if (rightPos < 0 && rightSp > 0) { // ��ɓ���
		rightGoalPos = rightPos + rightStartPos;
	}
	leftSp = abs(leftSp);
	rightSp = abs(rightSp);
}

void Tank::stop(const StopMode mode) {
	switch (mode) {
	case StopMode::BRAKE:
		leftMotor->setVelocity(0);
		rightMotor->setVelocity(0);
		break;
	case StopMode::HOLD:
		setPosition(getLeftEncoder(), getRightEncoder(), 0, 0, unit::rad,false, false);
		break;
	}
}

void Tank::setDireciton(double direction, double speed, const unit unit)
{
	if (unit == unit::rad) direction = pd_rad_to_degrees(direction);
	cout << "direction: " << direction << endl;
	double startAngle = gyro.getGyro();
	speed = abs(speed);
	if (speed > maxVelocity) speed = maxVelocity;
	
	// �܂��͉����������߂�
	uint8_t minus = 0;
	if (direction > startAngle) {
		if (abs(direction - startAngle) > 180) {
			// ���v��� 
			leftMotor->setPosition(INFINITY);
			rightMotor->setPosition(-1 * INFINITY);
			cout << "right1" << endl;
			minus = 1;
		}
		else {
			// �����v���
			rightMotor->setPosition(INFINITY);
			leftMotor->setPosition(-1 * INFINITY);
			cout << "left2" << endl;
		}
	}
	else {
		if (abs(direction - startAngle) > 180) {
			// �����v��� 
			rightMotor->setPosition(INFINITY);
			leftMotor->setPosition(-1 * INFINITY);
			cout << "left3" << endl;
			minus = 2;
		}
		else {
			// ���v���
			leftMotor->setPosition(INFINITY);
			rightMotor->setPosition(-1 * INFINITY);
			cout << "right4" << endl;
		}
	}

	// �� PID ��
	double Kp = 0.05; // �Ԃ����Ⴏ�O�����N�\���Ȃ�����P����ł���
	double Ki = 0;
	double Kd = 0;
	double u, error = 100, last_error = 0;
	while (1) {
		if (robot->step(timeStep) == -1 || abs(error) < 0.5 || abs(error) > 359.5) break;
		double angle = gyro.getGyro();
		cout << "angle: " << angle << endl;
		error = direction - angle;
		if (minus == 1) { // �����ƌ������@������񂾂낤�ȂƎv������v��Ȃ�������
			if (angle < direction) error -= 360;
			else minus = 0;
		}
		else if (minus == 2) {
			if (angle > direction) error += 360;
			else minus = 0;
		}
		cout << "error: " << error << endl;
		u = Kp * error + Ki * (error + last_error) + Kd * (error - last_error) ;
		if (u > 1) u = 1;
		else if (u < -1) u = -1;
		leftMotor->setVelocity(-1 * speed * u);
		rightMotor->setVelocity(speed * u);
	}
	stop(StopMode::HOLD);
	robot->step(160);
}

void Tank::gpsTraceSimple(const GPSPosition& goal, const double speed, const Direction_of_Travel& direction)
{
	// ���ݒn���擾
	GPSPosition presentPos = gps.getPosition();
	double angle_to_goal = pd_rad_to_degrees( atan2(goal.z - presentPos.z, goal.x - presentPos.x) ); // �ړI�n�ւ̕Ίp
	cout << "angle_to_goal: " << angle_to_goal << endl;
	// ���݂̊p�x���擾
	double presentAngle = gyro.getGyro();
	double angle_to_O = pd_angle(angle_to_goal);
	if (abs(angle_to_O - presentAngle) > 10) {
		cout << "angle_to_0: " << angle_to_O << endl;
		setDireciton(angle_to_O, speed);
	}
}

static double pd_angle(double angle) {
	if (angle > 360) angle = 360;
	else if (angle < 0) angle = 0;

	angle = 360 - angle; // �������]
	if (angle < 270) {
		return angle + 90;
	}
	else return angle - 270;
}