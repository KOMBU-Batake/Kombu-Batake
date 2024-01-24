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
  cout << "----------------" << endl;
  cout << mapper.startTile_A.x << "," << mapper.startTile_A.z << endl;
  WallState front, back, right, left;
  mapper.getAroundWallState(mapper.currentTile_R,front, back, right, left);
  cout << "front:" << (int)front << endl;
  cout << "back:" << (int)back << endl;
  cout << "right:" << (int)right << endl;
  cout << "left:" << (int)left << endl;

  // èIóπÉRÉ}ÉìÉh
  char message = 'E';
  emitter->send(&message, 1);
  //while (robot->step(timeStep) != -1);

  delete robot;
  return 0; // ÇŒÇ¢ÇŒÇ¢Å`
}
