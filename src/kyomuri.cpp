// File:          kyomuri.cpp
// Date:
// Description:   For WRS
// Author:        koki0517
// Modifications:

#include <iostream>
#include <thread>
#include <mutex>
#include "../lib/devices.h"

using namespace webots;
using namespace std;

// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  enableDevices();
  // ÉÅÉCÉìÉãÉEÉEÉEÉEÉDÉDÉDÉDÉDÉDÉv

  cout << "start" << endl;
  mapper.map_A[0][1] = "1";
  mapper.map_A[4][3] = "S";
  mapper.printMap();
  cout << "----------------" << endl;
  mapper.markAroundWall(WallState::WALL, WallState::cneterWALL, WallState::rightWALL, WallState::leftWALL);
  mapper.printMap();
  cout << "----------------" << endl;
  mapper.updatePostion(0, -1);
  mapper.markTileAs(mapper.currentTile_R, TileState::OTHER);
  mapper.markAroundWall(WallState::noWALL, WallState::WALL, WallState::rightWALL, WallState::leftWALL);
  mapper.printMap();
  //lidar.updateLiDAR();
  //cout << "front; " << (int)lidar.isWall(LiDAR_degree::FRONT) << endl;
  //cout << "left; " << (int)lidar.isWall(LiDAR_degree::LEFT) << endl;
  //cout << "left half; " << (int)lidar.isWall(LiDAR_degree::LEFT_HALF) << endl;
  //cout << "right; " << (int)lidar.isWall(LiDAR_degree::RIGHT) << endl;
  //cout << "back; " << (int)lidar.isWall(LiDAR_degree::BACK) << endl;
  //cout << "----------------" << endl;
  //cout << "front-left; " << (int)lidar.isWall(LiDAR_degree::FRONT_LEFT) << endl;
  //cout << "front-right; " << (int)lidar.isWall(LiDAR_degree::FRONT_RIGHT) << endl;
  //cout << "================" << endl;
  //tank.setDireciton(90,3);
  //lidar.updateLiDAR();
  //cout << "front; " << (int)lidar.isWall(LiDAR_degree::FRONT) << endl;
  //cout << "left; " << (int)lidar.isWall(LiDAR_degree::LEFT) << endl;
  //cout << "right; " << (int)lidar.isWall(LiDAR_degree::RIGHT) << endl;
  //cout << "back; " << (int)lidar.isWall(LiDAR_degree::BACK) << endl;
  //cout << "back half; " << (int)lidar.isWall(LiDAR_degree::BACK_HALF) << endl;
  while (robot->step(timeStep) != -1);

  delete robot;
  return 0; // ÇŒÇ¢ÇŒÇ¢Å`
}
