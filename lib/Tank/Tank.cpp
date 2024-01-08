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

void Tank::setDireciton(double direction, double maxspeed, const unit unit)
{
	if (unit == unit::rad) direction = pd_rad_to_degrees(direction);
	cout << "direction: " << direction << endl;
	double startAngle = gyro.getGyro();
	maxspeed = abs(maxspeed);
	if (maxspeed > maxVelocity) maxspeed = maxVelocity;
	
	// �܂��͉����������߂�
	uint8_t minus = 0;
	if (direction > startAngle) {
		if (abs(direction - startAngle) > 180) {
			// ���v��� 
			leftMotor->setPosition(INFINITY);
			rightMotor->setPosition(-1 * INFINITY);
			setV_goal_right = goalPosition::minusINFINITY;
			setV_goal_left = goalPosition::plusINFINITY;
			//cout << "right1" << endl;
			minus = 1;
		}
		else {
			// �����v���
			rightMotor->setPosition(INFINITY);
			leftMotor->setPosition(-1 * INFINITY);
			setV_goal_right = goalPosition::plusINFINITY;
			setV_goal_left = goalPosition::minusINFINITY;
			//cout << "left2" << endl;
		}
	}
	else {
		if (abs(direction - startAngle) > 180) {
			// �����v��� 
			rightMotor->setPosition(INFINITY);
			leftMotor->setPosition(-1 * INFINITY);
			setV_goal_right = goalPosition::plusINFINITY;
			setV_goal_left = goalPosition::minusINFINITY;
			//cout << "left3" << endl;
			minus = 2;
		}
		else {
			// ���v���
			leftMotor->setPosition(INFINITY);
			rightMotor->setPosition(-1 * INFINITY);
			setV_goal_right = goalPosition::minusINFINITY;
			setV_goal_left = goalPosition::plusINFINITY;
			//cout << "right4" << endl;
		}
	}

	// �� PID ��
	double Kp = 0.05; // �Ԃ����Ⴏ�O�����N�\���Ȃ�����P����ł���
	double Ki = 0;
	double Kd = 0;
	double u, error = 100, last_error = 0;
	while (1) {
		if (robot->step(timeStep) == -1 || abs(error) < 0.1 || abs(error) > 359.9) break;
		double angle = gyro.getGyro();
		//cout << "angle: " << angle << endl;
		error = direction - angle;
		if (minus == 1) { // �����ƌ������@������񂾂낤�ȂƎv������v��Ȃ�������
			if (angle < direction) error -= 360;
			else minus = 0;
		}
		else if (minus == 2) {
			if (angle > direction) error += 360;
			else minus = 0;
		}
		//cout << "error: " << error << endl;
		u = Kp * error + Ki * (error + last_error) + Kd * (error - last_error) ;
		if (u > 1) u = 1;
		else if (u < -1) u = -1;
		leftMotor->setVelocity(-1 * maxspeed * u);
		rightMotor->setVelocity(maxspeed * u);
		last_error = error;
	}
	stop(StopMode::HOLD);
	robot->step(160);
}

void Tank::gpsTraceSimple(const GPSPosition& goal, double speed, Direction_of_Travel direction, const StopMode stopmode)
{
	speed = abs(speed);
	if (speed > maxVelocity) speed = maxVelocity;
	// ���ݒn���擾
	GPSPosition presentPos = gps.getPosition(), presentPosRAW;
	double dz = goal.z - presentPos.z;
	double dx = goal.x - presentPos.x;
	double angle_to_goal = pd_rad_to_degrees( atan2(dz, dx) ); // �ړI�n�ւ̕Ίp
	cout << "angle_to_goal: " << angle_to_goal << endl;
	// ���݂̊p�x���擾
	double presentAngle = gyro.getGyro();
	double angle_to_O = pd_angle(angle_to_goal);
	if (abs(angle_to_O - presentAngle) > 10) { // �ړI�n�ւ̊p�x�ƌ��݂̊p�x��10�x�ȏジ��Ă����� �␳����
		cout << "angle_to_0: " << angle_to_O << endl;
		setDireciton(angle_to_O, speed);
	}

	// PID����������!!
	double error_x = 0, last_error_x = 0;
	double error_z = 0, last_error_z = 0;
	double u_x, u_z;
	double leftSpeed = 0, rightSpeed = 0;
	setV_goal_right = goalPosition::plusINFINITY;
	setV_goal_left = goalPosition::plusINFINITY;
	leftMotor->setPosition(INFINITY);
	rightMotor->setPosition(INFINITY);
	if (direction == Direction_of_Travel::z) {
		double referenceX = goal.x;
		if (stopmode == StopMode::BRAKE || stopmode == StopMode::HOLD) {

			// �y����������
			error_z = goal.z - presentPos.z;
			if (error_z > 0) direction = Direction_of_Travel::z_plus; // Z����
			else direction = Direction_of_Travel::z_minus; // Z����

			// PID
			double Kp_x = 0.5, Ki_x = 0.01, Kd_x = 15;
			double Kp_z = 0.6, Ki_z = 0, Kd_z = 0;
			while (1) {
				presentPosRAW = gps.getPositionRAW();
				presentPos = gps.filter(presentPosRAW);
				// �w�������̂���
				if (direction == Direction_of_Travel::z_plus) error_x = referenceX - presentPosRAW.x;
				else error_x = presentPosRAW.x - referenceX; // z_plus
				u_x = Kp_x * error_x + Ki_x * (error_x + last_error_x) + Kd_x * (error_x - last_error_x);
				// Z�������̖ړI�n�ւ̋���
				error_z = abs(goal.z - presentPos.z);
				u_z = Kp_z * error_z + Ki_z * (error_z + last_error_z) + Kd_z * (error_z - last_error_z);
				if (u_z > 1) u_z = 1;

				leftSpeed = speed * u_z + u_x;
				rightSpeed = speed * u_z - u_x;
				if (leftSpeed > maxVelocity) leftSpeed = maxVelocity;
				if (rightSpeed > maxVelocity) rightSpeed = maxVelocity;
				leftMotor->setVelocity(leftSpeed);
				rightMotor->setVelocity(rightSpeed);
				cout << "error_z: " << error_z << ", u_z: " << u_z << ", error_x: " << error_x << ", u_x: " << u_x << endl;

				last_error_x = error_x;
				last_error_z = error_z;
				if (abs(error_z) < 0.1 || robot->step(timeStep) == -1) break;
			}
			stop(stopmode);
		}
		else { // coast
			// ���񂪂���̂��邢
		}
	}
	else if (direction == Direction_of_Travel::x) {
		double referenceZ = goal.z;
		if (stopmode == StopMode::BRAKE || stopmode == StopMode::HOLD) {
			// �w����������
			error_x = goal.x - presentPos.x;
			if (error_x > 0) direction = Direction_of_Travel::x_plus; // �w����
			else direction = Direction_of_Travel::x_minus; // �w����

			// PID
			double Kp_x = 0.6, Ki_x = 0, Kd_x = 0;
			double Kp_z = 0.5, Ki_z = 0.01, Kd_z = 15;
			while (1) {
				presentPosRAW = gps.getPositionRAW();
				presentPos = gps.filter(presentPosRAW);
				// Z�������̂���
				if (direction == Direction_of_Travel::x_plus) error_z = referenceZ - presentPosRAW.z;
				else error_z = presentPosRAW.z - referenceZ; // x_plus
				u_z = Kp_z * error_z + Ki_z * (error_z + last_error_z) + Kd_z * (error_z - last_error_z);

				// �w�������̖ړI�n�ւ̋���
				error_x = abs(goal.x - presentPos.x);
				u_x = Kp_x * error_x + Ki_x * (error_x + last_error_x) + Kd_x * (error_x - last_error_x);
				if (u_x > 1) u_x = 1;

				leftSpeed = speed * u_x + u_z;
				rightSpeed = speed * u_x - u_z;
				if (leftSpeed > maxVelocity) leftSpeed = maxVelocity;
				if (rightSpeed > maxVelocity) rightSpeed = maxVelocity;
				leftMotor->setVelocity(leftSpeed);
				rightMotor->setVelocity(rightSpeed);
				cout << "error_x: " << error_x << ", u_x: " << u_x << ", error_z: " << error_z << ", u_z: " << u_z << endl;

				last_error_x = error_x;
				last_error_z = error_z;
				if (abs(error_x) < 0.1 || robot->step(timeStep) == -1) break;
			}
			stop(stopmode);
		}
		else { // coast

		}
	}
	else { // diagonal

	}
}

static double pd_angle(double angle) {
	if (angle > 360) angle = 360;
	else if (angle < 0) angle += 360;

	angle = 360 - angle; // �������]
	if (angle < 270) {
		return angle + 90;
	}
	else return angle - 270;
}