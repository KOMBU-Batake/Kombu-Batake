#pragma once

#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <iostream>

using namespace webots;
using namespace std;

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

enum Colors {
	WHITE,
	BLACK,
	OTHERS,
};

class ColorSensor
{
public:
	ColorSensor(webots::Camera* camera){ this->colorsen = camera; } //  colorCamってグローバル変数だからいちいちこんなことしなくてもいい希ガス

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
			hsv.hue = 1024;
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

	int getColor() {
	}

	ColorRGB RGB = { 0,0,0 };
private:
	webots::Camera* colorsen;
	// 落とし穴と普通の床、チェックポイントタイルの定義は別で特別に行ふ
	ColorRange blueRange = { 240, 5, 77, 5, 248, 5 }; // 許容誤差はとりま5 実験では2未満
	ColorRange purpleRange = { 268, 5, 74, 5, 214, 5 };
	ColorRange swampRange= { 40, 5, 53, 5, 197, 5 };
};

