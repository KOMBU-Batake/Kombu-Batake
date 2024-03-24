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

/* DFS�Ŏg���}�b�v
 * BFS�ɂ��g����
 * 5*5�̃^�C���x�[�X
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

	void addNorth(const int& i = 1); // �k�����Ƀ^�C����ǉ�
	void addSouth(const int& i = 1); // ������Ƀ^�C����ǉ�
	void addWest(const int& j = 1); // �������Ƀ^�C����ǉ�
	void addEast(const int& j = 1); // �������Ƀ^�C����ǉ�

	vector<MapAddress> getRoute(MapAddress& Goal, const vector<MapAddress>& stack_of_DFS);

	void markAroundStatus(const aroundStatus& status, const double& angle, const bool& isFirst = false);

	void printMap() {
		MapAddress addr_List = convertRtoListPoint(currentTile_R);
// �}�b�v�̕\��
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

	/* �Ȃ񂾃R�� */
	int convertTiletoList(int tile) {
		return tile * 4;
	}

	/* ��΍��W�𑊑΍��W�ɕϊ� */
	MapAddress convertAtoR(const MapAddress& addr_A) {
		return { addr_A.x - startTile_A.x + 1, addr_A.z - startTile_A.z + 1 };
	}

	/* ���΍��W���΍��W�ɕϊ� */
	MapAddress convertRtoA(const MapAddress& addr_R) {
		return { addr_R.x + startTile_A.x - 1, addr_R.z + startTile_A.z - 1 };
	}

	/* �^�C�����W���烊�X�g�̃^�C�����S�̍��W�ɕϊ� */
	MapAddress convertAtoListPoint(const MapAddress& addr_A) {
		return { addr_A.x * 2, addr_A.z * 2 };
	}

	/* ���X�g�̃^�C�����S�̍��W����^�C�����W�ɕϊ� */
	MapAddress convertListPointtoA(const MapAddress& addr_list) {
		return { (addr_list.x) / 2, (addr_list.z) / 2 };
	}

	/* ���΍��W���烊�X�g�̃^�C�����S�̍��W�ɕϊ� */
	MapAddress convertRtoListPoint(const MapAddress& addr_R) {
		return convertAtoListPoint(convertRtoA(addr_R));
	}

	/* ���X�g�̃^�C�����S�̍��W���瑊�΍��W�ɕϊ� */
	MapAddress convertListPointtoR(const MapAddress& addr_list) {
		return convertAtoR(convertListPointtoA(addr_list));
	}
};