#include <iostream>
#include <webots/Robot.hpp>
#include <iomanip>
#include <math.h>
#include <vector>
#include <deque>
#include <algorithm>
#include <map>
#include <unordered_set>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>

extern webots::Robot *robot;
extern int timeStep;

using namespace webots;
using namespace std;

/* DFSで使うマップ
 * BFSにも使うよ
 * 5*5のタイルベース
 */

struct MapAddress {
	int x;
	int z;

	bool operator==(const MapAddress& other) const {
		return x == other.x && z == other.z;
	}

	bool operator!=(const MapAddress& other) const {
		return x != other.x || z != other.z;
	}
};

struct aroundStatus {
	bool front = false;
	bool left = false;
	bool right = false;
	bool back = false;
	bool front_left = false;
	bool front_right = false;
};

class MapperS {
public:
	void updatePostion(const int& dx_R, const int& dz_R);

	void addNorth(const int& i = 1); // 北方向にタイルを追加
	void addSouth(const int& i = 1); // 南方向にタイルを追加
	void addWest(const int& j = 1); // 西方向にタイルを追加
	void addEast(const int& j = 1); // 東方向にタイルを追加

	vector<MapAddress> getRoute(MapAddress& Goal, const vector<MapAddress>& stack_of_DFS);

	void markAroundStatus(const aroundStatus& status, const double& angle, const bool& isFirst = false);

	void printMap() {
		MapAddress addr_List = convertRtoListPoint(currentTile_R);
// マップの表示
		for (int i = 0; i < map_S.size(); i++) {
			for (int j = 0; j < map_S[0].size(); j++) {
				if (j == addr_List.x && i == addr_List.z) {
					cout << std::setw(2) << "R  ";
				}
				else {
					std::cout << std::setw(2) << map_S[i][j] << " ";
				}
			}
			cout << "\n";
		}
		cout << endl;
	}
private:
	vector<vector<int>> map_S = vector<vector<int>>(5, vector<int>(5, 0));

	MapAddress startTile_A = { 1, 1 }, startTile_R = { 1, 1 };
	MapAddress currentTile_A = { 1, 1 }, currentTile_R = { 1, 1 };

	void resetSearchPoint();

	deque<MapAddress> canGoNEWS(const MapAddress& addr_A, const int& num, const int& cond = -1, bool cond2 = true);

	/* なんだコレ */
	int convertTiletoList(int tile) {
		return tile * 4;
	}

	/* 絶対座標を相対座標に変換 */
	MapAddress convertAtoR(const MapAddress& addr_A) {
		return { addr_A.x - startTile_A.x + 1, addr_A.z - startTile_A.z + 1 };
	}

	/* 相対座標を絶対座標に変換 */
	MapAddress convertRtoA(const MapAddress& addr_R) {
		return { addr_R.x + startTile_A.x - 1, addr_R.z + startTile_A.z - 1 };
	}

	/* タイル座標からリストのタイル中心の座標に変換 */
	MapAddress convertAtoListPoint(const MapAddress& addr_A) {
		return { addr_A.x * 2, addr_A.z * 2 };
	}

	/* リストのタイル中心の座標からタイル座標に変換 */
	MapAddress convertListPointtoA(const MapAddress& addr_list) {
		return { (addr_list.x) / 2, (addr_list.z) / 2 };
	}

	/* 相対座標からリストのタイル中心の座標に変換 */
	MapAddress convertRtoListPoint(const MapAddress& addr_R) {
		return convertAtoListPoint(convertRtoA(addr_R));
	}

	/* リストのタイル中心の座標から相対座標に変換 */
	MapAddress convertListPointtoR(const MapAddress& addr_list) {
		return convertAtoR(convertListPointtoA(addr_list));
	}
};