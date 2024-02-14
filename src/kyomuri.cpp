#include <iostream>
#include "../lib/devices.h"

int main(int argc, char **argv) {
  enableDevices();

  cout << "start" << endl;

  // [‚³—Dæ’Tõ
  //DFS();
  lidar2.update(gps.expectedPos);
  NcmPoints points = lidar2.getNcmPoints(LiDAR_degree::FRONT, 10);
  cout << "left count : " << points.model_left.size() << " right count : " << points.model_right.size() << endl;
  points.model_left.insert(points.model_left.end(), points.model_right.begin(), points.model_right.end());
  for(auto p : points.model_left) {
		cout << p.x << " " << p.z << endl;
	}
  lidar2.getWallType(LiDAR_degree::FRONT);

  cout << "end" << endl;

  delete robot;
  return 0; // ‚Î‚¢‚Î‚¢`
}