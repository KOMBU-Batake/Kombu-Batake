//#pragma once

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

#include "../lib/ColorSensor/ColorSensor2.h"
#include "../lib/IMU/IMU.h"
#include "../lib/GlobalPositioningSystem/GlobalPositioningSystem.h"
#include "../lib/Tank/Tank.h"
#include "../lib/PointCloudLiDAR/PointCloudLiDAR.h"
#include "../lib/PointCloudLiDAR/RecognizingSpace/RecognizingSpace.h"
#include "../src/Map/Map.h"
#include "../src/DFS/DFS.h"
#include "../lib/MyCam/MyCam.h"
#include "../lib/PointCloudLiDAR/LiDAR2.h"

/* デバイスの設定、以下略をするよ */

using namespace webots;
using namespace std;
using namespace cv;

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
ColorSensor2 colorsensor;
GyroZ gyro;
GlobalPositioningSystem gps;
Tank tank(leftMotor, rightMotor, leftEncoder, rightEncoder);
Map mapper;
PointCloudLiDAR pcLiDAR;
MyCam myCam;
LiDAR2 lidar2;
//MyMath myMath;

void delay(int ms) {
	float initTime = (float)robot->getTime();	// Store starting time (in seconds)
	while (robot->step(timeStep) != -1) {
		if ((robot->getTime() - initTime) * 1000.0 > ms) { // If time elapsed (converted into ms) is greater than value passed in
			return;
		}
	}
}

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

	//Mat inputImage = Mat(leftCam->getHeight(), leftCam->getWidth(), CV_8UC4, (void*)leftCam->getImage());
	//imwrite("leftCam.png", inputImage);
	//std::vector<cv::KeyPoint> keypoints_source, keypoints_target;
	//cv::Ptr<cv::Feature2D> f2d = cv::KAZE::create();

	//Mat targetImage = imread("../../protos/textures/placard-8-corrosive.png", cv::IMREAD_COLOR);
	//Mat img_gray_source, img_gray_target;
	//cvtColor(inputImage, img_gray_source, cv::COLOR_BGR2GRAY);
	//cvtColor(targetImage, img_gray_target, cv::COLOR_BGR2GRAY);

	//cv::Mat canny;
	//int thresh = 100;
	//cv::Canny(img_gray_source, canny, thresh, thresh * 2);

	//// cv::findContours は第一引数を破壊的に利用するため imshow 用に別変数を用意しておきます。
	//cv::Mat canny2 = canny.clone();

	//// cv::Point の配列として、輪郭を計算します。
	//std::vector<std::vector<cv::Point> > contours;
	//std::vector<cv::Vec4i> hierarchy;
	//cv::findContours(canny, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	//std::cout << contours.size() << std::endl; //=> 36
	//std::cout << contours[contours.size() - 1][0] << std::endl; //=> [154, 10]

	//cv::Mat drawing = cv::Mat::zeros(canny.size(), CV_8UC3);
	//cv::RNG rng(12345);

	//for (size_t i = 0; i < contours.size(); i++) {
	//	cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
	//	cv::drawContours(drawing, contours, (int)i, color);
	//}
	//cv::imshow("drawing", drawing);
	//imwrite("srawing.png", drawing);

}

