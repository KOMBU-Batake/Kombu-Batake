#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <webots/Camera.hpp>
#include <webots/Robot.hpp>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>
#include <cstring>
#include "../PointCloudLiDAR/PointCloudLiDAR.h"
#include "../GlobalPositioningSystem/GlobalPositioningSystem.h"

using namespace webots;
using namespace std;
using namespace cv;

extern Robot* robot;
//extern PointCloudLiDAR pcLiDAR;
extern Receiver* receiver;
extern Emitter* emitter;
extern Camera* leftCam;
extern Camera* rightCam;
extern GlobalPositioningSystem gps;

extern int timeStep;

class MyCam {
public:
	MyCam(){}

	void updateLeft() {
		Mat tmp(leftCam->getHeight(), leftCam->getWidth(), CV_8UC4, (void*)leftCam->getImage());
		cvtColor(tmp, tmp, cv::COLOR_BGRA2BGR);
		cvtColor(tmp, Leftframe, cv::COLOR_BGR2HSV);
		if (Leftframe.empty()) {
			std::cout << "Error: left frame is empty" << std::endl;
		}
	}

	void updateRight() {
		Mat tmp(rightCam->getHeight(), rightCam->getWidth(), CV_8UC4, (void*)rightCam->getImage());
		cvtColor(tmp, tmp, cv::COLOR_BGRA2BGR);
		cvtColor(tmp, Rightframe, cv::COLOR_BGR2HSV);
		if (Rightframe.empty()) {
			std::cout << "Error: right frame is empty" << std::endl;
		}
	}

	void update() {
		//lidar.updateLiDAR();
		//updateLeft();
		//updateRight();
	}

	void detectAndReportLeft() {
		//if (abs(lidar.getDIstanceSimple(384))<6) {
		//	// O ============================================================================================
		//	cv::Scalar lowerb2 = cv::Scalar(40 * 0.5, 120, 78);  // 下限(H, S, V)
		//	cv::Scalar upperb2 = cv::Scalar(70 * 0.5, 255, 255); // 上限(H, S, V)

		//	// 閾値を満たすピクセルを抽出
		//	cv::Mat mask2;
		//	cv::inRange(Leftframe, lowerb2, upperb2, mask2);

		//	// ピクセル数をカウント
		//	int count2 = cv::countNonZero(mask2);
		//	std::cout << "Number of pixels LO: " << count2 << std::endl;
		//	if (count2 > 50) {
		//		delay(1300);
		//		char message[9]; // Here we use a 9 byte array, since sizeof(int + int + char) = 9
		//		GPSPosition pos = gps.getPosition(); // Get the current gps position of the robot
		//		int victim_pos[2] = { (int)round(pos.x), (int)round(pos.z) };
		//		cout << "Victim position: " << victim_pos[0] << " " << victim_pos[1] << endl;
		//		memcpy(message, victim_pos, sizeof(victim_pos)); // Copy the victim position into the message array
		//		message[8] = 'O'; // The victim type is harmed
		//		emitter->send(message, sizeof(message));
		//		robot->step(timeStep);
		//		return;
		//	}

		//	// F ============================================================================================
		//	cv::Scalar lowerb = cv::Scalar(300*0.5, 102, 78);  // 下限(H, S, V)
		//	cv::Scalar upperb = cv::Scalar(359 * 0.5, 255, 255); // 上限(H, S, V)

		//	// 閾値を満たすピクセルを抽出
		//	cv::Mat mask;
		//	cv::inRange(Leftframe, lowerb, upperb, mask);

		//	// ピクセル数をカウント
		//	int count = cv::countNonZero(mask);
		//	if (count > 50) {
		//		delay(1300);
		//		char message[9]; // Here we use a 9 byte array, since sizeof(int + int + char) = 9
		//		GPSPosition pos = gps.getPosition(); // Get the current gps position of the robot
		//		int victim_pos[2] = { (int)round(pos.x), (int)round(pos.z)};
		//		memcpy(message, victim_pos, sizeof(victim_pos)); // Copy the victim position into the message array
		//		message[8] = 'F'; // The victim type is harmed
		//		emitter->send(message, sizeof(message));
		//		robot->step(timeStep);
		//	}
		//}
	}

	void detectAndReportRight() {
		//if (abs(lidar.getDIstanceSimple(128)) < 6) {
		//	// O ============================================================================================
		//	cv::Scalar lowerb2 = cv::Scalar(40 * 0.5, 120, 78);  // 下限(H, S, V)
		//	cv::Scalar upperb2 = cv::Scalar(70 * 0.5, 255, 255); // 上限(H, S, V)

		//	// 閾値を満たすピクセルを抽出
		//	cv::Mat mask2;
		//	cv::inRange(Rightframe, lowerb2, upperb2, mask2);

		//	// ピクセル数をカウント
		//	int count2 = cv::countNonZero(mask2);
		//	std::cout << "Number of pixels RO: " << count2 << std::endl;
		//	if (count2 > 50) {
		//		delay(1300);
		//		char message[9]; // Here we use a 9 byte array, since sizeof(int + int + char) = 9
		//		GPSPosition pos = gps.getPosition(); // Get the current gps position of the robot
		//		int victim_pos[2] = { (int)round(pos.x), (int)round(pos.z) };
		//		cout << "Victim position: " << victim_pos[0] << " " << victim_pos[1] << endl;
		//		memcpy(message, victim_pos, sizeof(victim_pos)); // Copy the victim position into the message array
		//		message[8] = 'O'; // The victim type is harmed
		//		emitter->send(message, sizeof(message));
		//		robot->step(timeStep);
		//		return;
		//	}

		//	// F ============================================================================================
		//	cv::Scalar lowerb = cv::Scalar(300 * 0.5, 102, 78);  // 下限(H, S, V)
		//	cv::Scalar upperb = cv::Scalar(359 * 0.5, 255, 255); // 上限(H, S, V)

		//	// 閾値を満たすピクセルを抽出
		//	cv::Mat mask;
		//	cv::inRange(Rightframe, lowerb, upperb, mask);

		//	// ピクセル数をカウント
		//	int count = cv::countNonZero(mask);
		//	if (count > 50) {
		//		delay(1300);
		//		//std::cout << "Number of pixels: " << count << std::endl;
		//		char message[9]; // Here we use a 9 byte array, since sizeof(int + int + char) = 9

		//		GPSPosition pos = gps.getPosition(); // Get the current gps position of the robot
		//		int victim_pos[2] = { (int)round(pos.x), (int)round(pos.z) };
		//		//cout << "Victim position: " << victim_pos[0] << " " << victim_pos[1] << endl;

		//		memcpy(message, victim_pos, sizeof(victim_pos)); // Copy the victim position into the message array
		//		message[8] = 'F'; // The victim type is harmed
		//		emitter->send(message, sizeof(message));
		//		robot->step(timeStep);
		//	}
		//}
	}

	void saveLeftImage(string filename) {
		leftCam->saveImage(filename, 100);
	}
	void saveRightImage(string filename) {
		rightCam->saveImage(filename, 100);
	}

	Mat Leftframe,Rightframe;
private:
	void delay(int ms) {
		float initTime = (uint8_t)robot->getTime();	// Store starting time (in seconds)
		while (robot->step(timeStep) != -1) {
			if ((robot->getTime() - initTime) * 1000.0 > ms) { // If time elapsed (converted into ms) is greater than value passed in
				return;
			}
		}
	}
};