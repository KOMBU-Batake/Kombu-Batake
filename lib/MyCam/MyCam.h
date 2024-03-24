#pragma once
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <webots/Camera.hpp>
#include <webots/Robot.hpp>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>
#include <cstring>
#include "../GlobalPositioningSystem/GlobalPositioningSystem.h"
#include "../ColorSensor/ColorSensor.h"

using namespace webots;
using namespace std;
using namespace cv;

extern Robot* robot;
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
		leftInputImage = Mat(leftCam->getHeight(), leftCam->getWidth(), CV_8UC4, (void*)leftCam->getImage());
		//cvtColor(tmp, tmp, cv::COLOR_BGRA2BGR);
		//cvtColor(tmp, Leftframe, cv::COLOR_BGR2HSV);
	}

	void updateRight() {
		rightInputImage = Mat(rightCam->getHeight(), rightCam->getWidth(), CV_8UC4, (void*)rightCam->getImage());
		//cvtColor(tmp, tmp, cv::COLOR_BGRA2BGR);
		//cvtColor(tmp, Rightframe, cv::COLOR_BGR2HSV);
	}

	void update() {
		updateLeft();
		updateRight();
	}

	void test() {
		int count = 0;
		for (int i = 10; i <= 57; i++) {
			ColorHSV hsv = convertRGBtoHSV(leftInputImage.at<cv::Vec4b>(30, i));
			if (hsv.value > 150) count++;
			cout << hsv.value << endl;
		}
		cout << count << endl;
	}

	string leftFindYellow(const float& distance) {
		if (distance > 9) return "n";
		bool foundRed = false;
		for (int i = 31; i <= 32; i++) {
			int redCount = 0;
			int yellowCount = 0;
			for (int j = 3; j <= 35; j++) {
				ColorHSV hsv = convertRGBtoHSV(leftInputImage.at<cv::Vec4b>(j, i));
				//cout << "Hue: " << hsv.hue << " Saturation: " << hsv.saturation << " Value: " << hsv.value << endl;
				if (hsv.hue > 340 && hsv.hue < 360 && hsv.value > 170) {
					cout << "Red found" << endl;
					cout << "hue:" << hsv.hue << " saturation: " << hsv.saturation << " value: " << hsv.value << endl;
					redCount++;
				}
				if (hsv.hue > 50 && hsv.hue < 60) {
					cout << "Yellow found" << endl;
					cout << "hue:" << hsv.hue << " saturation: " << hsv.saturation << " value: " << hsv.value << endl;
					yellowCount++;
				}
				else if (foundRed) cout << "hue:" << hsv.hue << " saturation: " << hsv.saturation << " value: " << hsv.value << endl;
			}
			if (redCount >= 2) {
				if (yellowCount >= 1) {
					cout << "O" << endl;
					imwrite("leftO.png", leftInputImage);
					return "O";
				}
				else {
					cout << "F" << endl;
					imwrite("leftF.png", leftInputImage);
					return "F";
				}
			}
		}

		//int count = 0;
		//for (int i = 10; i <= 57; i++) {
		//	ColorHSV hsv = convertRGBtoHSV(leftInputImage.at<cv::Vec4b>(9, i));
		//	if (hsv.value > 150) count++;
		//}
		//if (count > 15) {
		//	GPSPosition pos = gps.getPosition();
		//	if (victimStack.size() == 0) {
		//		victimStack.push_back(pos);
		//		cout << "left H " << distance << endl;
		//		return "H";
		//	}
		//	for (auto& past : victimStack) {
		//		if ((pow(pos.z - past.z, 2) + pow(pos.x - past.x, 2)) < 36) {
		//			return "n";
		//		}
		//	}
		//	victimStack.push_back(pos);
		//	cout << "left H " << distance << endl;
		//	return "H";
		//}

		return "n";
	}

	string rightFindYellow(const float& distance) {
		if (distance > 9) return "n";
		bool foundRed = false;
		for (int i = 31; i <= 32; i++) {
			int redCount = 0;
			int yellowCount = 0;
			for (int j = 3; j <= 35; j++) {
				ColorHSV hsv = convertRGBtoHSV(rightInputImage.at<cv::Vec4b>(j, i));
				//cout << "Hue: " << hsv.hue << " Saturation: " << hsv.saturation << " Value: " << hsv.value << endl;
				if (hsv.hue > 340 && hsv.hue < 360 && hsv.value > 170) {
					cout << "Red found" << endl;
					cout << "hue:" << hsv.hue << " saturation: " << hsv.saturation << " value: " << hsv.value << endl;
					redCount++;
				}
				if (hsv.hue > 50 && hsv.hue < 60) {
					cout << "Yellow found" << endl;
					cout << "hue:" << hsv.hue << " saturation: " << hsv.saturation << " value: " << hsv.value << endl;
					yellowCount++;
				}
				else if (foundRed) cout << "hue:" << hsv.hue << " saturation: " << hsv.saturation << " value: " << hsv.value << endl;
			}
			if (redCount >= 2) {
				if (yellowCount >= 1) {
					cout << "O" << endl;
					imwrite("rightO.png", rightInputImage);
					return "O";
				}
				else {
					cout << "F" << endl;
					imwrite("rightF.png", rightInputImage);
					return "F";
				}
			}
		}

		//int count = 0;
		//for (int i = 10; i <= 57; i++) {
		//	ColorHSV hsv = convertRGBtoHSV(rightInputImage.at<cv::Vec4b>(9, i));
		//	if (hsv.value > 150) count++;
		//}
		//if (count > 15) {
		//	GPSPosition pos = gps.getPosition();
		//	if (victimStack.size() == 0) {
		//		victimStack.push_back(pos);
		//		cout << "right H " << distance << endl;
		//		return "H";
		//	}
		//	for (auto& past : victimStack) {
		//		if ((pow(pos.z - past.z, 2) + pow(pos.x - past.x, 2)) < 36) {
		//			return "n";
		//		}
		//	}
		//	victimStack.push_back(pos);
		//	cout << "right H " << distance << endl;
		//	return "H";
		//}

		return "n";
	}

	vector<bool> leftHole() {
		vector<bool> result(2, false);
		float leftFloorBrightness = 0, leftFloorSaturation = 0;
		for (int i = 34; i <= 36; i++) {
			for (int j = 14; j <= 16; j++) {
				ColorHSV hsv = convertRGBtoHSV(leftInputImage.at<cv::Vec4b>(i, j));
				leftFloorBrightness += hsv.value;
				leftFloorSaturation += hsv.saturation;
			}
		}
		leftFloorBrightness /= 9;
		leftFloorSaturation /= 9;
		if (leftFloorSaturation < 0.1f && leftFloorBrightness < 35) {
			result[1] = true;
		}
		float leftFloorBrightness2 = 0, leftFloorSaturation2 = 0;
		for (int i = 34; i <= 36; i++) {
			for (int j = 47; j <= 49; j++) {
				ColorHSV hsv = convertRGBtoHSV(leftInputImage.at<cv::Vec4b>(i, j));
				leftFloorBrightness2 += hsv.value;
				leftFloorSaturation2 += hsv.saturation;
			}
		}
		leftFloorBrightness2 /= 9;
		leftFloorSaturation2 /= 9;
		if (leftFloorSaturation2 < 0.1f && leftFloorBrightness2 < 30) {
			result[0] = true;
		}
		return result;
	}

	vector<bool> rightHole() {
		vector<bool> result(2, false);
		float leftFloorBrightness = 0, leftFloorSaturation = 0;
		for (int i = 34; i <= 36; i++) {
			for (int j = 14; j <= 16; j++) {
				ColorHSV hsv = convertRGBtoHSV(rightInputImage.at<cv::Vec4b>(i, j));
				leftFloorBrightness += hsv.value;
				leftFloorSaturation += hsv.saturation;
			}
		}
		leftFloorBrightness /= 9;
		leftFloorSaturation /= 9;
		if (leftFloorSaturation < 0.1f && leftFloorBrightness < 30) {
			result[0] = true;
		}
		float leftFloorBrightness2 = 0, leftFloorSaturation2 = 0;
		for (int i = 34; i <= 36; i++) {
			for (int j = 47; j <= 49; j++) {
				ColorHSV hsv = convertRGBtoHSV(rightInputImage.at<cv::Vec4b>(i, j));
				leftFloorBrightness2 += hsv.value;
				leftFloorSaturation2 += hsv.saturation;
			}
		}
		leftFloorBrightness2 /= 9;
		leftFloorSaturation2 /= 9;
		if (leftFloorSaturation2 < 0.1f && leftFloorBrightness2 < 35) {
			result[1] = true;
		}
		return result;
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
	Mat leftInputImage, rightInputImage;

	void findSquares(const Mat& image, vector<vector<Point> >& squares);
private:
	void delay(int ms) {
		float initTime = (uint8_t)robot->getTime();	// Store starting time (in seconds)
		while (robot->step(timeStep) != -1) {
			if ((robot->getTime() - initTime) * 1000.0 > ms) { // If time elapsed (converted into ms) is greater than value passed in
				return;
			}
		}
	}

	ColorHSV convertRGBtoHSV(Vec4b pixel) {
		ColorHSV hsv = { 512.0,0,0 };
		ColorRGB RGB = { pixel[2], pixel[1], pixel[0] };
		float max = std::max(std::max((float)pixel[0], (float)pixel[1]), (float)pixel[2]);
		float min = std::min(std::min((float)pixel[0], (float)pixel[1]), (float)pixel[2]);
		float diff = (float)(max - min);
		if (diff == 0) {
			hsv.hue = 0;
		}
		else if (max == RGB.red) {
			hsv.hue = 60 * ((float)(RGB.green - RGB.blue) / diff);
		}
		else if (max == RGB.green) {
			hsv.hue = 60 * ((float)(RGB.blue - RGB.red) / diff) + 120;
		}
		else if (max == RGB.blue) {
			hsv.hue = 60 * ((float)(RGB.red - RGB.green) / diff) + 240;
		}
		if (hsv.hue < 0) {
			hsv.hue += 360;
		}
		hsv.saturation = 100 * diff / max;
		hsv.value = max;
		return hsv;
	}

	double angle(Point pt1, Point pt2, Point pt0)
	{
		double dx1 = pt1.x - pt0.x;
		double dy1 = pt1.y - pt0.y;
		double dx2 = pt2.x - pt0.x;
		double dy2 = pt2.y - pt0.y;
		return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
	}

	int thresh = 50, N = 11;
	const char* wndname = "Square Detection Demo";
	vector<GPSPosition> victimStack;
};