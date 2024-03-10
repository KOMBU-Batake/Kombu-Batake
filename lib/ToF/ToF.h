#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>

using namespace webots;
using namespace std;

/* �� �| �I �� �� �Z �� �T �� �x �� �g �� �� */
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