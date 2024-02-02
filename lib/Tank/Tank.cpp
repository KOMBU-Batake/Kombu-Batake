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
	//cout << "direction: " << direction << endl;
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

// �o�ł�!!�T�C�N���}�e�B�b�N���G�x53�̗�!!
bool Tank::gpsTrace(const GPSPosition& goal, double speed, const StopMode stopmode, int timeout_ms, Direction_of_Travel direction)
{
	speed = abs(speed);
	if (speed > maxVelocity) speed = maxVelocity;
	// ���ݒn���擾
	GPSPosition presentPos = gps.getPosition(), presentPosRAW;
	double dz = goal.z - presentPos.z;
	double dx = goal.x - presentPos.x;
	double angle_to_goal = pd_rad_to_degrees( atan2(dz, dx) ); // �ړI�n�ւ̕Ίp
	//cout << "angle_to_goal: " << angle_to_goal << endl;
	// ���݂̊p�x���擾
	double presentAngle = gyro.getGyro();
	double angle_to_O = pd_angle(angle_to_goal);
	if (abs(angle_to_O - presentAngle) > 10) { // �ړI�n�ւ̊p�x�ƌ��݂̊p�x��10�x�ȏジ��Ă����� �␳����
		//cout << "angle_to_0: " << angle_to_O << endl;
		setDireciton(angle_to_O, speed);
	}
	// �����̏C��
  if (angle_to_O < 3 || angle_to_O > 357 || (angle_to_O > 177 && angle_to_O < 183)) {
		direction = Direction_of_Travel::z; // �y�������Ɍ���Ȃ��߂�
		//cout << "z" << endl;
	}
    else if (((87 < angle_to_O) && (angle_to_O < 93)) || ((267 < angle_to_O) && (angle_to_O < 273))) {
		direction = Direction_of_Travel::x; // �w�������Ɍ���Ȃ��߂�
		//cout << "x" << endl;
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
	// coast�̏ꍇ�͐i�s�����ւ̋�����p����PID���s��Ȃ�

	if (direction == Direction_of_Travel::z) { // �y������ ====================================================================================================
		double referenceX = goal.x;
		// �y����������
		error_z = goal.z - presentPos.z;
		if (error_z > 0) direction = Direction_of_Travel::z_plus; // Z����
		else direction = Direction_of_Travel::z_minus; // Z����

		// PID
		double Kp_x = 0.5, Ki_x = 0.01, Kd_x = 15;
		double Kp_z = 0.6, Ki_z = 0, Kd_z = 0;
		while (1) {
			if (!checkColor()) {
				stop(StopMode::HOLD);
				return false;
			}
			presentPosRAW = gps.getPositionRAW();
			presentPos = gps.filter(presentPosRAW);
			// �w�������̂���
			if (direction == Direction_of_Travel::z_minus) error_x = referenceX - presentPosRAW.x;
			else error_x = presentPosRAW.x - referenceX; // z_plus
			u_x = Kp_x * error_x + Ki_x * (error_x + last_error_x) + Kd_x * (error_x - last_error_x);

			error_z = abs(goal.z - presentPos.z);
			if (stopmode == StopMode::BRAKE || stopmode == StopMode::HOLD) {
				// Z�������̖ړI�n�ւ̋���
				u_z = Kp_z * error_z + Ki_z * (error_z + last_error_z) + Kd_z * (error_z - last_error_z);
				if (u_z > 1) u_z = 1;
			} else u_z = 1;

			leftSpeed = (speed + u_x) * u_z;
			rightSpeed = (speed - u_x) * u_z;
			if (leftSpeed > maxVelocity) leftSpeed = maxVelocity;
			if (rightSpeed > maxVelocity) rightSpeed = maxVelocity;
			if (leftSpeed  < (-1* maxVelocity)) leftSpeed = -1 * maxVelocity;
			if (rightSpeed < (-1* maxVelocity)) rightSpeed = -1 * maxVelocity;
			leftMotor->setVelocity(leftSpeed);
			rightMotor->setVelocity(rightSpeed);
			//cout << "error_z: " << error_z << ", u_z: " << u_z << ", error_x: " << error_x << ", u_x: " << u_x << endl;

			last_error_x = error_x;
			last_error_z = error_z;
			if (abs(error_z) < 0.05 || robot->step(timeStep) == -1) break;
		}
		if (stopmode == StopMode::BRAKE || stopmode == StopMode::HOLD) stop(stopmode);
	}
	else if (direction == Direction_of_Travel::x) { // �w������ ====================================================================================================
		double referenceZ = goal.z;
		// �w����������
		error_x = goal.x - presentPos.x;
		if (error_x > 0) direction = Direction_of_Travel::x_plus; // �w����
		else direction = Direction_of_Travel::x_minus; // �w����

		// PID
		double Kp_x = 0.6, Ki_x = 0, Kd_x = 0;
		double Kp_z = 0.5, Ki_z = 0.01, Kd_z = 15;
		while (1) {
			if (!checkColor()) {
				stop(StopMode::HOLD);
				return false;
			}
			presentPosRAW = gps.getPositionRAW();
			presentPos = gps.filter(presentPosRAW);
			// Z�������̂���
			if (direction == Direction_of_Travel::x_plus) error_z = referenceZ - presentPosRAW.z;
			else error_z = presentPosRAW.z - referenceZ; // x_plus
			u_z = Kp_z * error_z + Ki_z * (error_z + last_error_z) + Kd_z * (error_z - last_error_z);

			error_x = abs(goal.x - presentPos.x);
			if (stopmode == StopMode::BRAKE || stopmode == StopMode::HOLD) {
				// �w�������̖ړI�n�ւ̋���
				u_x = Kp_x * error_x + Ki_x * (error_x + last_error_x) + Kd_x * (error_x - last_error_x);
				if (u_x > 1) u_x = 1;
			} else u_x = 1;

			leftSpeed = (speed + u_z) * u_x;
			rightSpeed = (speed - u_z) * u_x;
			if (leftSpeed > maxVelocity) leftSpeed = maxVelocity;
			if (rightSpeed > maxVelocity) rightSpeed = maxVelocity;
			if (leftSpeed < (-1 * maxVelocity)) leftSpeed = -1 * maxVelocity;
			if (rightSpeed < (-1 * maxVelocity)) rightSpeed = -1 * maxVelocity;
			leftMotor->setVelocity(leftSpeed);
			rightMotor->setVelocity(rightSpeed);
			//cout << "error_x: " << error_x << ", u_x: " << u_x << ", error_z: " << error_z << ", u_z: " << u_z << endl;

			last_error_x = error_x;
			last_error_z = error_z;
			if (abs(error_x) < 0.05 || robot->step(timeStep) == -1) break;
		}
		if (stopmode == StopMode::BRAKE || stopmode == StopMode::HOLD) stop(stopmode);
	}
	else { // �΂ߕ��� ===================================================================================================================================================
		// az-x+c=0 �̎��̒萔�����߂�
		double a = (goal.x - presentPos.x) / (goal.z - presentPos.z);
		double c = -1* a * presentPos.z + presentPos.x; 
		double sqrt_ac = sqrt(a*a + 1); // sqrt(a^2 + 1) �_�ƒ����̋��������Ŏg��

		// PID�̕ϐ�
		double Kp_d = 0.6, Ki_d = 0, Kd_d = 0;
		double Kp_vertialdir = 0.8, Ki_vertialdir = 0.01, Kd_vertialdir = 15;
		double error_d = 0, error_vertialdir = 0, last_error_d = 0, last_error_vertialdir = 0;
		double u_d, u_vertialdir;
		double dx_d, dz_d;
		
		// �i�ޕ�����Z����������
		Direction_of_Travel directionPorM = Direction_of_Travel::z_minus;
		if (goal.z > presentPos.z) directionPorM = Direction_of_Travel::z_plus;
		
		do {
			if (!checkColor()) {
				stop(StopMode::HOLD);
				return false;
			}
			presentPosRAW = gps.getPositionRAW();
			presentPos = gps.filter(presentPosRAW);
			dx_d = presentPos.x - goal.x;
			dz_d = presentPos.z - goal.z;
			error_d = sqrt(pow(dx_d,2) + pow(dz_d,2)); // presentPos��goal�̋���
			error_vertialdir = (a * presentPos.z - presentPos.x + c) / sqrt_ac; // presentPos�ƒ����̋���
			
			if (stopmode == StopMode::BRAKE || stopmode == StopMode::HOLD) {
				u_d = Kp_d * error_d + Ki_d * (error_d + last_error_d) + Kd_d * (error_d - last_error_d);
				if (u_d > 1) u_d = 1;
			} else u_d = 1;
			u_vertialdir = Kp_vertialdir * error_vertialdir + Ki_vertialdir * (error_vertialdir + last_error_vertialdir) + Kd_vertialdir * (error_vertialdir - last_error_vertialdir);
			if (directionPorM == Direction_of_Travel::z_plus) u_vertialdir = -1 * u_vertialdir;
			leftSpeed = (speed + u_vertialdir) * u_d;
			rightSpeed = (speed - u_vertialdir) * u_d;
			if (leftSpeed > maxVelocity) leftSpeed = maxVelocity;
			if (rightSpeed > maxVelocity) rightSpeed = maxVelocity;
			leftMotor->setVelocity(leftSpeed);
			rightMotor->setVelocity(rightSpeed);
			//cout << "error_d: " << error_d << ", u_d: " << u_d << ", error_vertialdir: " << error_vertialdir << ", u_vertialdir: " << u_vertialdir << endl;

			last_error_d = error_d;
			last_error_vertialdir = error_vertialdir;
		} while (abs(dz_d) > 0.05 && abs(dx_d) > 0.05 && robot->step(timeStep) != -1);
		if (stopmode == StopMode::BRAKE || stopmode == StopMode::HOLD) stop(stopmode);
	}
	return true;
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