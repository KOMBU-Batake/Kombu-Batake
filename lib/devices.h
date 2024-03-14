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

/* デバイスの設定、以下略をするよ */

using namespace webots;
using namespace std;

// Robotインスタンスの作成
Robot* robot = new Robot();

// タイムステップの取得
int timeStep = (int)robot->getBasicTimeStep(); // =16

/* もぉたぁ */
Motor* leftMotor = robot->getMotor("leftWheel motor");
Motor* rightMotor = robot->getMotor("rightWheel motor");

/* ぽじしょんせんさぁ っていうか えんこぉだぁ */
PositionSensor* leftEncoder = leftMotor->getPositionSensor();
PositionSensor* rightEncoder = rightMotor->getPositionSensor();

/* らいだぁ */
Lidar* centralLidar = robot->getLidar("lidar");

/* からぁせんさぁ */
Camera* colorCam = robot->getCamera("frontCamera");

/* きゃめら */
Camera* leftCam = robot->getCamera("leftCamera");
Camera* rightCam = robot->getCamera("rightCamera");

/* じーぴーえす */
GPS* gpsXZ = robot->getGPS("gps");

/* あいえむゆぅ */
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
	
	// 余白の追加
	copyMakeBorder(inputImage, inputImage,  1, 0, 10, 10, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255, 0));

	// 出力画像を保存
	cv::imwrite("output_image.png", inputImage); // 例: "output_image.png" を適切なファイル名に置き換えてください

	//読み込んだ画像の四角形の頂点
	cv::Point2f pts1[] = { cv::Point2f(8, 28), cv::Point2f(75,28), cv::Point2f(56, 0), cv::Point2f(27, 0) };
	//出力画像に対応する四角形の頂点
	cv::Point2f pts2[] = { cv::Point2f(24, 36), cv::Point2f(60, 36), cv::Point2f(60, 0), cv::Point2f(24, 0) };
	//射影変形行列を代入
	cv::Mat pmat = cv::getPerspectiveTransform(pts1, pts2);
	//射影変換第３引数は変換行列
	cv::warpPerspective(inputImage, inputImage, pmat, inputImage.size(), cv::INTER_LINEAR);

	cv::imwrite("output_image1.png", inputImage); // 例: "output_image.png" を適切なファイル名に置き換えてください
}