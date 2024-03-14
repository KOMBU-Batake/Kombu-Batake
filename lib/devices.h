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
#include "../lib/Tank/Tank.h"
#include "../lib/PointCloudLiDAR/PointCloudLiDAR.h"
#include "../lib/PointCloudLiDAR/RecognizingSpace/RecognizingSpace.h"
//#include "../lib/myMath/myMath.h"
#include "../src/Map/Map.h"
#include "../src/DFS/DFS.h"
#include "../lib/MyCam/MyCam.h"
#include "../lib/PointCloudLiDAR/LiDAR2.h"

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

/* ���炟���񂳂� */
Camera* colorCam = robot->getCamera("frontCamera");

/* ����߂� */
Camera* leftCam = robot->getCamera("leftCamera");
Camera* rightCam = robot->getCamera("rightCamera");

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
Tank tank(leftMotor, rightMotor, leftEncoder, rightEncoder);
Map mapper;
PointCloudLiDAR pcLiDAR;
MyCam myCam;
LiDAR2 lidar2;
//MyMath myMath;

void enableDevices() {
	leftEncoder->enable(timeStep);
	rightEncoder->enable(timeStep);
	centralLidar->enable(timeStep);
	colorCam->enable(timeStep);
	leftCam->enable(timeStep);
	rightCam->enable(timeStep);
	IMU->enable(timeStep);
	gpsXZ->enable(timeStep);
	receiver->enable(timeStep);
	robot->step(timeStep); // delay 16ms
	gps.recoedStartPosition();
	
	colorCam->setFov(1.5f);
	robot->step(timeStep); // delay 16ms

	Mat inputImage(colorCam->getHeight(), colorCam->getWidth(), CV_8UC4, (void*)colorCam->getImage());
	colorCam->saveImage("1.png", 100);

	array<cv::Vec4b, 4> floorColor = { inputImage.at<cv::Vec4b>(30, 39), inputImage.at<cv::Vec4b>(31, 39), inputImage.at<cv::Vec4b>(32, 39), inputImage.at<cv::Vec4b>(33, 39) };
	array<float,4> floorColorAverage;
	for (int j = 0; j < 4; j++) {
		for (int i = 0; i < 4; i++) floorColorAverage[j] += (float)floorColor[i][j];
		floorColorAverage[j] /= 4;
	}
	cout << "floorColorAverage: " << (int)floorColorAverage[0] << " " << (int)floorColorAverage[1] << " " << (int)floorColorAverage[2] << " " << (int)floorColorAverage[3] << endl;

	cout << "RGB: " << (int)inputImage.at<cv::Vec4b>(10, 20)[0] << " " << (int)inputImage.at<cv::Vec4b>(10, 20)[1] << " " << (int)inputImage.at<cv::Vec4b>(10, 20)[2] << endl;
	
	// �]���̒ǉ�
	copyMakeBorder(inputImage, inputImage,  1, 0, 10, 10, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255, 0));

	// �o�͉摜��ۑ�
	cv::imwrite("output_image.png", inputImage); // ��: "output_image.png" ��K�؂ȃt�@�C�����ɒu�������Ă�������

	//�ǂݍ��񂾉摜�̎l�p�`�̒��_
	cv::Point2f pts1[] = { cv::Point2f(8, 28), cv::Point2f(75,28), cv::Point2f(56, 0), cv::Point2f(27, 0) };
	//�o�͉摜�ɑΉ�����l�p�`�̒��_
	cv::Point2f pts2[] = { cv::Point2f(24, 36), cv::Point2f(60, 36), cv::Point2f(60, 0), cv::Point2f(24, 0) };
	//�ˉe�ό`�s�����
	cv::Mat pmat = cv::getPerspectiveTransform(pts1, pts2);
	//�ˉe�ϊ���R�����͕ϊ��s��
	cv::warpPerspective(inputImage, inputImage, pmat, inputImage.size(), cv::INTER_LINEAR);

	cv::imwrite("output_image1.png", inputImage); // ��: "output_image.png" ��K�؂ȃt�@�C�����ɒu�������Ă�������
}