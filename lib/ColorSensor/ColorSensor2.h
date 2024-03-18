#pragma once

/* フルサイズカメラ用のクラス
 * ColorSensorクラスを継承している 
 */

#include <numeric>
#include "ColorSensor.h"
using namespace cv;

struct obstacleState {
	vector<XZcoordinate> obstaclePositionsStart;
	vector<XZcoordinate> obstaclePositionsEnd;
	vector<XZcoordinate> closestPoint;
	bool leftHoleState;
	bool rightHoleState;

	obstacleState() : leftHoleState(false), rightHoleState(false) {}
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
	}

	TileState getLeftColor() {
		Vec4b centerPixel = inputImage.at<cv::Vec4b>(37, 26);
		ColorHSV hsv = convertRGBtoHSV(centerPixel);
		Colors color = selectColor(hsv);
		if (color != Colors::OTHERS) return tileColorMap[color];

		// 灰色系の処理
		vector<uchar> leftFloorBrightness(25);
		for (int i = 35; i <= 39; i++) {
			for (int j = 24; j <= 28; j++) {
				Vec4b pixel = inputImage.at<cv::Vec4b>(i, j);
				leftFloorBrightness[(i - 35) * 5 + (j - 24)] = max(pixel[0], max(pixel[1], pixel[2]));
			}
		}
		double variance = calculateVariance(leftFloorBrightness);
		if (variance > 10) return TileState::CHECKPOINT;
		if (hsv.value < 35 || centerPixel == Vec4b{ 106,106,106 }) return TileState::HOLE;
		return TileState::OTHER;
	}

	TileState getRightColor() {
		Vec4b centerPixel = inputImage.at<cv::Vec4b>(37, 37);
		ColorHSV hsv = convertRGBtoHSV(centerPixel);
		Colors color = selectColor(hsv);
		if (color != Colors::OTHERS) return tileColorMap[color];

		// 灰色系の処理
		vector<uchar> rightFloorBrightness(25);
		for (int i = 35; i <= 39; i++) {
			for (int j = 35; j <= 39; j++) {
				Vec4b pixel = inputImage.at<cv::Vec4b>(i, j);
				rightFloorBrightness[(i - 35) * 5 + (j - 35)] = max(pixel[0], max(pixel[1], pixel[2]));
			}
		}
		double variance = calculateVariance(rightFloorBrightness);
		if (variance > 10) return TileState::CHECKPOINT;
		if (hsv.value < 35 || centerPixel == Vec4b{106,106,106}) return TileState::HOLE;
		return TileState::OTHER;
	}

	obstacleState obstacle();

private:
	void nextTile(obstacleState& state, vector<int>& numbers);

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

	Colors selectColor(ColorHSV hsv) {
		if (isTheColor(blueRange, hsv)) {
			return Colors::BLUE;
		}
		else if (isTheColor(purpleRange, hsv)) {
			return Colors::PURPLE;
		}
		else if (isTheColor(swampRange, hsv)) {
			return Colors::SWAMP;
		}
		else if (isTheColor(greenRange, hsv)) {
			return Colors::GREEN;
		}
		else if (isTheColor(redRange, hsv)) {
			return Colors::RED;
		}
		else {
			return Colors::OTHERS;
		}
	}

	double calculateVariance(const std::vector<uchar>& data) {
		// 平均
		double sum = std::accumulate(data.begin(), data.end(), 0.0);
		double mean = sum / data.size();

		// 各要素と平均の差の二乗を合計
		double squaredDifferencesSum = 0.0;
		for (const double& value : data) {
			double difference = value - mean;
			squaredDifferencesSum += difference * difference;
		}

		// 分散を返す
		return squaredDifferencesSum / data.size();
	}

	Mat inputImage, transformedImage;

	ColorRange blueRange = { 240, 5, 80, 5, 232, 5 };
	ColorRange purpleRange = { 266, 5, 75, 5, 185, 5 };
	ColorRange swampRange = { 40, 5, 53, 5, 166, 5 };
	ColorRange greenRange = { 120, 5, 89, 5, 225, 5 };
	ColorRange redRange = { 0, 5, 80, 5, 233, 5 };
};

static bool operator==(const Vec4b& lhs, const Vec4b& rhs) {
	return lhs[0] == rhs[0] && lhs[1] == rhs[1] && lhs[2] == rhs[2];
}