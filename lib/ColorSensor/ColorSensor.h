#pragma once

#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <iostream>
#include <algorithm>
#include <map>

using namespace webots;
using namespace std;

enum class TileState {
	OTHER = 0,
	WALL = 1,
	HOLE = 2,
	SWAMP = 3,
	CHECKPOINT = 4,
	START = 5,
	AREA1to2 = 6,
	AREA2to3 = 7,
	AREA3to4 = 8,
	AREA1to4 = 9,
	UNKNOWN, // = "-
	visited, // LiDARのクラスでは使わない
};

typedef struct
{
	int red;
	int green;
	int blue;
} ColorRGB;

typedef struct
{
	float hue;
	float saturation;
	float value;
} ColorHSV;

typedef struct {
	int hue; // hueの理想的な値
	int hue_diff; // hueの許容誤差
	int saturation; // 以下同文
	int saturation_diff;
	int value;
	int value_diff;
} ColorRange;

enum class Colors {
	WHITE,
	BLACK,
	CHECKPOINT,
	BLUE,
	PURPLE,
	GREEN,
	RED,
	SWAMP,
	OTHERS,
};

class ColorSensor
{
public:
	ColorSensor(webots::Camera* camera){ this->colorsen = camera; } // colorCamをexternすればいちいちこんなことしなくてもよくね？

	/* これを実行しないとクラス内のどの関数でも値が更新されないままなので注意 */
	void update() {
		const unsigned char* rgb = colorsen->getImage();
		RGB.red = rgb[2];
		RGB.green = rgb[1];
		RGB.blue = rgb[0];
	}
	uint8_t getRed() { return RGB.red; }
	uint8_t getGreen() { return RGB.green; }
	uint8_t getBlue() { return RGB.blue; }

	ColorHSV getHSV() {
		ColorHSV hsv = {512.0,0,0};
		uint8_t max = std::max(RGB.red, std::max(RGB.green, RGB.blue));
		uint8_t min = std::min(RGB.red, std::min(RGB.green, RGB.blue));
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

	Colors getColor() {
		/* HSVベースで床の色を判断する */
		ColorHSV hsv = getHSV();
		if (hsv.value < 35) {
			cout << "under 35 hue: " << hsv.hue << " sat: " << hsv.saturation << " val: " << hsv.value << endl;
			return Colors::SWAMP;
		}
		else if (hsv.value < 50) {
			cout << "hue: " << hsv.hue << " sat: " << hsv.saturation << " val: " << hsv.value << endl;
			return Colors::BLACK;
		}
		else if (hsv.saturation < 1 && hsv.value > 230) {
			return Colors::CHECKPOINT;
		}
		else if (hsv.saturation < 1 && hsv.value > 200) {
			return Colors::WHITE;
		}
		else if (isTheColor(blueRange, hsv)){
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
		else
		{
			return Colors::OTHERS;
		}
	}

	TileState getTileColor() {
		Colors col = getColor();
		return tileColorMap[col];
	}

	ColorRGB RGB = { 0,0,0 };
private:
	webots::Camera* colorsen;
	// 落とし穴と普通の床、チェックポイントタイルの定義は別で特別に行ふ
	ColorRange blueRange = { 240, 5, 77, 5, 248, 5 }; // 許容誤差はとりま5 実験では2未満
	ColorRange purpleRange = { 268, 5, 74, 5, 214, 5 };
	ColorRange swampRange= { 40, 5, 53, 5, 197, 5 };
	ColorRange greenRange = { 120, 5, 87, 5, 244, 5 };
	ColorRange redRange = { 0, 5, 77, 5, 248, 5 };

	bool isTheColor(const ColorRange& range, const ColorHSV& hsv) {
		if (abs(hsv.hue - range.hue) <= range.hue_diff && abs(hsv.saturation - range.saturation) <= range.saturation_diff && abs(hsv.value - range.value) <= range.value_diff) {
			return true;
		}
		else {
			return false;
		}
	}

	std::map<Colors, TileState> tileColorMap = {
		{Colors::WHITE, TileState::OTHER},
		{Colors::BLACK, TileState::HOLE},
		{Colors::CHECKPOINT, TileState::CHECKPOINT},
		{Colors::BLUE, TileState::AREA1to2},
		{Colors::PURPLE, TileState::AREA2to3},
		{Colors::GREEN,  TileState::AREA1to4},
		{Colors::RED, TileState::AREA3to4},
		{Colors::SWAMP, TileState::SWAMP},
	};
};
