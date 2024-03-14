#pragma once
#include "ColorSensor.h"
using namespace cv;

enum class HoleState {
	NO_HOLE,
	LEFT_HOLE,
	RIGHT_HOLE,
	BOTH_HOLE,
};

class ColorSensor2 :
  public ColorSensor
{
public:
	int Height, Width;

	ColorSensor2() : Height(colorCam->getHeight()), Width(colorCam->getWidth()){
		colorCam->setFov(1.5f);
	}

	void update() {
		inputImage = Mat(Height, Width, CV_8UC4, (void*)colorCam->getImage());
		array<cv::Vec4b, 4> floorColor = { inputImage.at<cv::Vec4b>(30, 39), inputImage.at<cv::Vec4b>(31, 39), inputImage.at<cv::Vec4b>(32, 39), inputImage.at<cv::Vec4b>(33, 39) };
		array<float, 4> floorColorAverage;
		for (int j = 0; j < 4; j++) {
			for (int i = 0; i < 4; i++) floorColorAverage[j] += (float)floorColor[i][j];
			floorColorAverage[j] /= 4;
		}
		cout << "floorColorAverage: " << (int)floorColorAverage[0] << " " << (int)floorColorAverage[1] << " " << (int)floorColorAverage[2] << " " << (int)floorColorAverage[3] << endl;

		cout << "RGB: " << (int)inputImage.at<cv::Vec4b>(10, 20)[0] << " " << (int)inputImage.at<cv::Vec4b>(10, 20)[1] << " " << (int)inputImage.at<cv::Vec4b>(10, 20)[2] << endl;

	}

	TileState getLeftColor() {
		return TileState::OTHER;
	}

	TileState getRightColor() {
		return TileState::OTHER;
	}

	XZcoordinate obstacle();

	Mat inputImage;
};