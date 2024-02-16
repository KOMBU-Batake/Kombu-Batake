#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>

using namespace webots;
using namespace std;

/* 圧 倒 的 欠 陥 セ ン サ 二 度 と 使 う か */
class ToFSensor
{
public:
	ToFSensor(DistanceSensor* tof) {
		this->ToF = tof;
	}
	double getDistanceCM() {
		return ToF->getValue() * 100;
	}
private:
	DistanceSensor* ToF;
};