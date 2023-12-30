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
	int hue; // hue�̗��z�I�Ȓl
	int hue_diff; // hue�̋��e�덷
	int saturation; // �ȉ�����
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
	ColorSensor(webots::Camera* camera){ this->colorsen = camera; } //  colorCam���ăO���[�o���ϐ������炢����������Ȃ��Ƃ��Ȃ��Ă�������K�X

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
	// ���Ƃ����ƕ��ʂ̏��A�`�F�b�N�|�C���g�^�C���̒�`�͕ʂœ��ʂɍs��
	ColorRange blueRange = { 240, 5, 77, 5, 248, 5 }; // ���e�덷�͂Ƃ��5 �����ł�2����
	ColorRange purpleRange = { 268, 5, 74, 5, 214, 5 };
	ColorRange swampRange= { 40, 5, 53, 5, 197, 5 };
};

