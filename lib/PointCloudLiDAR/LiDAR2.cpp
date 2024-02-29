#include "LiDAR2.h"

NcmPoints LiDAR2::getNcmPoints(const LiDAR_degree& direction, float range) {
  // 中央10cmのデータを取得
  NcmPoints ncmP;
  int center = getCenterNum(direction);
  std::cout << "center: " << center << endl;
  ncmP.centerNum = center;
  float half_range = range / 2;

  if (direction == LiDAR_degree::LEFT || direction == LiDAR_degree::RIGHT) {
    float centralZ = pointCloud[center].z;
    // 左方向
    int leftStart = center;
    for (int i = center; i > center - 80; i--) {
      if (abs(readPoint(i).z - centralZ) <= half_range) leftStart = i;
    }
    // 右方向
    int rightEnd = center + 1;
    for (int i = center; i < center + 80; i++) {
      if (abs(readPoint(i).z - centralZ) <= half_range) rightEnd = i;
    }
    cout << "leftStart: " << leftStart << " rightEnd: " << rightEnd << endl;
  }
  else { // FTONT or BACK
    float centralX = pointCloud[center].x;
    // 左方向
    int leftStart = center;
    for (int i = center; i > center - 80; i--) {
      if (abs(readPoint(i).x - centralX) <= half_range) leftStart = i;
    }
    // 右方向
    int rightEnd = center + 1;
    for (int i = center + 1; i < center + 80; i++) {
      if (abs(readPoint(i).x - centralX) <= half_range) rightEnd = i;
    }
    cout << "leftStart: " << leftStart << " rightEnd: " << rightEnd << endl;
  }

    if (direction == LiDAR_degree::LEFT || direction == LiDAR_degree::RIGHT) {
        ncmP.model_left = { pointCloud[center] };
        float centralZ = pointCloud[center].z;
        // 右方向
        ncmP.count_right = 0;
        while (1) {
            ncmP.count_right++;
            if (abs(readPoint(center + ncmP.count_right).z - centralZ) <= half_range) {
                ncmP.model_right.push_back(readPoint(center + ncmP.count_right));
            }
            else {
                ncmP.count_right--;
                break;
            }
        }
        // 左方向 
        ncmP.count_left = 0;
        while (1) {
            ncmP.count_left++;
            if (abs(centralZ - readPoint(center - ncmP.count_left).z) <= half_range) {
                ncmP.model_left.push_back(readPoint(center - ncmP.count_left));
                std::rotate(ncmP.model_left.rbegin(), ncmP.model_left.rbegin() + 1, ncmP.model_left.rend());
            }
            else {
                ncmP.count_left--;
                break;
            }
        }
    }
    else // FRONT or BACK
    {
        ncmP.model_left = { pointCloud[center] };
        float centralX = pointCloud[center].x;
        // 右方向
        ncmP.count_right = 0;
        while (1) {
            ncmP.count_right++;
            if (abs(readPoint(center + ncmP.count_right).x - centralX) <= half_range) {
                ncmP.model_right.push_back(readPoint(center + ncmP.count_right));
            }
            else {
                ncmP.count_right--;
                break;
            }
        }
        // 左方向 
        ncmP.count_left = 0;
        while (1) {
            ncmP.count_left++;
            if (abs(centralX - readPoint(center - ncmP.count_left).x) <= half_range) {
                ncmP.model_left.push_back(readPoint(center - ncmP.count_left));
                std::rotate(ncmP.model_left.rbegin(), ncmP.model_left.rbegin() + 1, ncmP.model_left.rend());
            }
            else {
                ncmP.count_left--;
                break;
            }
        }
    }
    return ncmP;
}

int LiDAR2::getCenterNum(LiDAR_degree direction, XZcoordinate centralPos) {
  // 中心の決定
  int center = 0;
  auto closest = pointCloud.begin();
  auto closest2 = pointCloud.end();
  switch (direction) { // もっと高速にはできるはずだよね、あとで考える
  case LiDAR_degree::FRONT:
    closest = min_element(pointCloud.begin(), pointCloud.begin() + 128, [](XZcoordinate s1, XZcoordinate s2) {
      return (abs(s1.x) < abs(s2.x)) && s1.z > 0;
      });
    closest2 = min_element(pointCloud.begin() + 384, pointCloud.end(), [](XZcoordinate s1, XZcoordinate s2) {
      return (abs(s1.x) < abs(s2.x)) && s1.z > 0;
      });
    if (abs(closest->x) > abs(closest2->x))  center = (int)std::distance(pointCloud.begin(), closest2);
    else center = (int)std::distance(pointCloud.begin(), closest);
    break;
  case LiDAR_degree::BACK:
    closest = min_element(pointCloud.begin() + 128, pointCloud.begin() + 384, [](XZcoordinate s1, XZcoordinate s2) {
      return (abs(s1.x) < abs(s2.x)) && s1.z < 0;
      });

    center = (int)std::distance(pointCloud.begin(), closest);
    break;
  case LiDAR_degree::LEFT:
    closest = min_element(pointCloud.begin() + 256, pointCloud.end(), [](XZcoordinate s1, XZcoordinate s2) {
      return (abs(s1.z) < abs(s2.z)) && s1.x < 0;
      });

    center = (int)std::distance(pointCloud.begin(), closest);
    break;
  case LiDAR_degree::RIGHT:
    closest = min_element(pointCloud.begin(), pointCloud.begin() + 256, [](XZcoordinate s1, XZcoordinate s2) {
      return (abs(s1.z) < abs(s2.z)) && s1.x > 0;
      });

    center = (int)std::distance(pointCloud.begin(), closest);
    break;
  default:
    break;
  }
  return center;
}

WallSet LiDAR2::getWallType(const LiDAR_degree& direction)
{
  WallSet wallSet = {WallType::typeNo, WallType::center_n, WallType::typeNo};
    // 中央10cmのデータを取得
    NcmPoints pointsSet = getNcmPoints(direction, 12);
    printLeftRight(pointsSet);
    cout << "-------------------" << endl;
    rotateToFront(pointsSet.model_left, direction);
    rotateToFront(pointsSet.model_right, direction);
    //MAXandMIN max_min = getMAX_MIN(pointsSet, direction);
    vector<int> featurePoints = VectorTracer(pointsSet);

    wallSet.left = identifyLeft(pointsSet, featurePoints);
    cout << "left: " << (int)wallSet.left << endl;
    wallSet.right = identifyRight(pointsSet, featurePoints);
    cout << "right: " << (int)wallSet.right << endl;
    wallSet.center = identifyCenter(pointsSet, wallSet, featurePoints);

    return wallSet;
}

WallType LiDAR2::identifyLeft(NcmPoints& pointSet, vector<int>& featurePoints)
{
  // 左側の壁をカット
  XZrange rangeToCut = {-4,-6.1f,18,0};
  int start = 0;
  for (auto& num: featurePoints) {
    if (rangeToCut.isIncluding(pointSet.read(num))) {
			start = num;
		}
	}
  cout << "LL start is: " << start << endl;
  start += 2;

  // 右側の壁をカット
  XZrange rangeToCut2 = { 0.1f, -2, 18, 0 };
  int end = pointSet.count_left;
  for (auto& num : featurePoints) {
    if (rangeToCut2.isIncluding(pointSet.read(num))) {
			end = num;
			cout << "LR end is: " << num << endl;
      break;
		}
	}
  end -= 2;

  // Z軸方向の最大値と最小値を取得
  float maxLeft = max_element(pointSet.model_left.begin()+start, pointSet.model_left.begin()+end,
    [](XZcoordinate a, XZcoordinate b) { return a.z < b.z; })->z;
  float minLeft = min_element(pointSet.model_left.begin()+start, pointSet.model_left.begin() + end,
    [](XZcoordinate a, XZcoordinate b) { return a.z < b.z; })->z;
  if (maxLeft > 18) return WallType::typeNo; // 壁がないと判断

  cout << "max: " << maxLeft << " min: " << minLeft << endl;

  // Z軸方向の平均
  float sumZ = 0.0f;
  for (auto it = pointSet.model_left.begin() + start; it != pointSet.model_left.begin() + end + 1; ++it) sumZ += it->z;
  // 2乗の平均
  float sumZpow2 = 0.0f;
  for (auto it = pointSet.model_left.begin() + start; it != pointSet.model_left.begin() + end + 1; ++it) sumZpow2 += (float)pow(it->z,2);
  // 分散
  float variance = (float)(sumZpow2 / (end - start + 1) - pow(sumZ / (end - start + 1), 2));
  cout << "variance: " << variance << endl;

  if (variance < 0.1f) { // まっすぐな壁
    if (maxLeft > 15) return WallType::type0;
    else if (maxLeft > 10) return WallType::type5;
    else return WallType::type10;
  }

  return WallType();
}

WallType LiDAR2::identifyRight(NcmPoints& pointSet, vector<int>& featurePoints)
{
  // 右側の壁をカット
  XZrange rangeToCut = { 4, 6.1f, 18, 0 };
  int end = pointSet.count_left + pointSet.count_right;
  for (auto& num : featurePoints) {
    if (rangeToCut.isIncluding(pointSet.read(num))) {
			end = num;
			cout << "RR end is: " << num << endl;
			break;
		}
	}
  end -= 2;
  end -= pointSet.count_left;

  // 左側の壁をカット
  XZrange rangeToCut2 = { 2, -0.1f, 18, 0 };
  int start = pointSet.count_left + 1;
  for (auto& num : featurePoints) {
    if (rangeToCut2.isIncluding(pointSet.read(num))) {
			start = num;
		}
  }
  cout << "RL start is: " << start << endl;
  start += 2;
  start -= pointSet.count_left;

  // Z軸方向の最大値と最小値を取得
  float maxRight = max_element(pointSet.model_right.begin() + start, pointSet.model_right.begin() + end,
    [](XZcoordinate a, XZcoordinate b) { return a.z < b.z; })->z;
  float minRight = min_element(pointSet.model_right.begin() + start, pointSet.model_right.begin() + end,
    [](XZcoordinate a, XZcoordinate b) { return a.z < b.z; })->z;
  if (maxRight > 18) return WallType::typeNo; // 壁がないと判断

  cout << "max: " << maxRight << " min: " << minRight << endl;

  // Z軸方向の平均
  float sumZ = 0.0f;
  for (auto it = pointSet.model_right.begin() + start; it != pointSet.model_right.begin() + end + 1; ++it) sumZ += it->z;
  // 2乗の平均
  float sumZpow2 = 0.0f;
  for (auto it = pointSet.model_right.begin() + start; it != pointSet.model_right.begin() + end + 1; ++it) sumZpow2 += (float)pow(it->z, 2);
  // 分散
  float variance = (float)(sumZpow2 / (end - start + 1) - pow(sumZ / (end - start + 1), 2));
  cout << "variance: " << variance << endl;

  if (variance < 0.1f) { // まっすぐな壁
  	if (maxRight > 15) return WallType::type0;
  	else if (maxRight > 10) return WallType::type5;
  	else return WallType::type10;
  }

  return WallType();
}

WallType LiDAR2::identifyCenter(NcmPoints& pointSet, const WallSet& wallset, vector<int>& featurePoints)
{
  return WallType::center_n;
}

static float getAngle(const XZcoordinate& p1, const XZcoordinate& p2, const XZcoordinate& p3) {
  XZcoordinate ab = {0,0}, ac = { 0,0 };
  ab.x = p1.x - p2.x;
  ab.z = p1.z - p2.z;
  ac.x = p3.x - p2.x;
  ac.z = p3.z - p2.z;
  XZcoordinate marged = ab + ac;
  return (float)(atan2(marged.z, marged.x) * 180 / M_PI);
}

vector<int> LiDAR2::VectorTracer(NcmPoints& pointSet)
{
	vector<vector<int>> featurePointsNum;
  vector<vector<float>> featurePointsTheta;
  float theta = 0, abstheta = 0;
  XZcoordinate p1, p2, p3;
  int start = pointSet.centerNum - pointSet.count_left;
  int last_updateed_i = 0;
  float last_angle = 0;
  for (int i = start; i <= pointSet.centerNum + pointSet.count_right; i++) {
    p1 = (readPoint(i - 1) + readPoint(i - 2)) / 2;
    p2 = readPoint(i);
    p3 = (readPoint(i + 1) + readPoint(i + 2)) / 2;
    theta = calculateAngle(p1, p2, p3);
    abstheta = abs(theta);
    if (abstheta <= 360 && abstheta > 240 || abstheta >= 0 && abstheta < 120) {
      if (i == last_updateed_i + 1 && (abs(last_angle - getAngle(p1, p2, p3)) < 45 || abs(last_angle - getAngle(p1, p2, p3)) > 315)) {
        featurePointsNum[featurePointsNum.size()-1].push_back(i - start);
        featurePointsTheta[featurePointsTheta.size()-1].push_back(theta);
        //cout << i << " " << theta << " " << getAngle(p1, p2, p3) << endl;
      }
      else {
				featurePointsNum.push_back({i - start});
        featurePointsTheta.push_back({ theta });
        //cout << "~~~~~~~~~" << endl << i << " " << theta << " " << getAngle(p1, p2, p3) << endl;
			}
      
      last_updateed_i = i;
      last_angle = getAngle(p1, p2, p3);
    }
	}

  vector<int> returnNum;
  for (size_t i = 0, size = featurePointsNum.size(); i < size; i++) {
    int minDistance = (int)std::distance(featurePointsTheta[i].begin(), min_element(featurePointsTheta[i].begin(), featurePointsTheta[i].end()));
    returnNum.push_back(featurePointsNum[i][minDistance]);
	}

  cout << "feature points -------------------" << endl;
  for (auto& num : returnNum) {
		cout << num+start << " " << num << endl;
	} 
  cout << "-------------------" << endl;
	return returnNum;
}

void LiDAR2::rotateToFront(vector<XZcoordinate>& points, LiDAR_degree direction) {
  // 都合のいいように座標を回転
  switch (direction) {
  case LiDAR_degree::BACK:
    transform(points.begin(), points.end(), points.begin(),
      [](XZcoordinate element) {
        element.x = -1 * element.x;
        element.z = -1 * element.z;
        return element;
      });
    break;
  case LiDAR_degree::LEFT:
    transform(points.begin(), points.end(), points.begin(),
      [](XZcoordinate element) {
        float tmp = element.x;
        element.x = -1 * element.z;
        element.z = -1 * tmp;
        return element;
      });
    break;
  case LiDAR_degree::RIGHT:
    transform(points.begin(), points.end(), points.begin(),
      [](XZcoordinate element) {
        float tmp = element.x;
        element.x = element.z;
        element.z = tmp;
        return element;
      });
    break;
  default:
    break;
  }
}

vector<XZcoordinate> LiDAR2::getNcmPoints(XZcoordinate center, const LiDAR_degree& direction, float range)
{
  vector<XZcoordinate> points;
  if (direction == LiDAR_degree::FRONT) {
    for (auto& point : pointCloud) {
      if (point.x > center.x - range / 2 && point.x < center.x + range / 2 && point.z > center.z) {
				points.push_back(point);
      }
    }
  }
  else if (direction == LiDAR_degree::BACK) {
    for (auto& point : pointCloud) {
      if (point.x > center.x - range / 2 && point.x < center.x + range / 2 && point.z < center.z) {
        points.push_back(point);
      }
    }
  }
  else if (direction == LiDAR_degree::LEFT) {
    for (auto& point : pointCloud) {
      if (point.z > center.z - range / 2 && point.z < center.z + range / 2 && point.x < center.x) {
				points.push_back(point);
			}
    }
  }
  else if (direction == LiDAR_degree::RIGHT) {
    for (auto& point : pointCloud) {
      if (point.z > center.z - range / 2 && point.z < center.z + range / 2 && point.x > center.x) {
        points.push_back(point);
      }
    }
  }

  return points;
}

static int8_t conditionFRONTandBACK(StraightLine line, float centerX, float half_range){
  if (line.x_min >= centerX - half_range && line.x_max <= centerX + half_range) return 1;
  if (line.x_min <= centerX - half_range && line.x_max >= centerX - half_range && line.x_max <= centerX + half_range) return 2;
  if (line.x_min >= centerX - half_range && line.x_min <= centerX + half_range && line.x_max >= centerX + half_range) return 3;
  if (line.x_min <= centerX - half_range && line.x_max >= centerX + half_range) return 4;
  return 0;
}

static int8_t conditionLEFTandRIGHT(StraightLine line, float centerZ, float half_range) {
  if (line.z_min >= centerZ - half_range && line.z_max <= centerZ + half_range) return 1;
  if (line.z_min <= centerZ - half_range && line.z_max >= centerZ - half_range && line.z_max <= centerZ + half_range) return 2;
  if (line.z_min >= centerZ - half_range && line.z_min <= centerZ + half_range && line.z_max >= centerZ + half_range) return 3;
  if (line.z_min <= centerZ - half_range && line.z_max >= centerZ + half_range) return 4;
  return 0;
}

vector<StraightLine> LiDAR2::getNcmLines(XZcoordinate center, const LiDAR_degree& direction, float range) {
  vector<StraightLine> returnlines;
  float half_range = range / 2;
  float centerX = 0, centerZ = 0;
  int index = 0;
  int8_t condition = 0;
  switch (direction) {
    case LiDAR_degree::FRONT:
    case LiDAR_degree::BACK:
      centerX = center.x;
      for (auto& line : lines) {
        condition = conditionFRONTandBACK(line, centerX, half_range);
        cout << "num: " << index << ", " <<  "cond: " << (int)condition << ", " << line.x_min << " " << line.x_max << " " << line.z_min << " " << line.z_max;
        if (condition &&
           (direction == LiDAR_degree::FRONT && line.z_max >= center.z ||
            direction == LiDAR_degree::BACK && line.z_min <= center.z)) {
          cout << " push";
          if (condition == 1) returnlines.push_back(line);
          else if (condition == 2) {
            StraightLine tmp = line;
            line.renewRangebyXmin(center.x-half_range);
            returnlines.push_back(tmp);
          }
          else if (condition == 3) {
						StraightLine tmp = line;
						line.renewRangebyXmax(center.x+half_range);
						returnlines.push_back(tmp);
          }
          else if (condition == 4) {
						StraightLine tmp = line;
						line.renewRangebyXmin(center.x-half_range);
						line.renewRangebyXmax(center.x+half_range);
						returnlines.push_back(tmp);
					}
        }
        index++;
        cout << endl;
      }
      break;
    case LiDAR_degree::LEFT:
    case LiDAR_degree::RIGHT:
      centerZ = center.z;
      for (auto& line : lines) {
        condition = conditionLEFTandRIGHT(line, centerZ, half_range);
        if (condition &&
           (direction == LiDAR_degree::LEFT && line.x_min < center.x ||
            direction == LiDAR_degree::RIGHT && line.x_max > center.x)) {
          if (condition == 1) returnlines.push_back(line);
          else if (condition == 2) {
						StraightLine tmp = line;
						line.renewRangebyZmin(center.z-half_range);
						returnlines.push_back(tmp);
					}
          else if (condition == 3) {
						StraightLine tmp = line;
						line.renewRangebyZmax(center.z+half_range);
						returnlines.push_back(tmp);
					}
          else if (condition == 4) {
						StraightLine tmp = line;
						line.renewRangebyZmin(center.z-half_range);
						line.renewRangebyZmax(center.z+half_range);
						returnlines.push_back(tmp);
					}
          cout << index << ", " << (int)condition;
        }
        index++;
      }
      break;
  }
  return lines;
}

MAXandMIN LiDAR2::getMAX_MIN(NcmPoints& pointsSet, LiDAR_degree direction)
{
    // 最大値と最小値を取得
    XZcoordinate maxLeft = *max_element(pointsSet.model_left.begin(), pointsSet.model_left.end(),
        [](XZcoordinate a, XZcoordinate b) { return a.z < b.z; });
    XZcoordinate minLeft = *min_element(pointsSet.model_left.begin(), pointsSet.model_left.end(),
        [](XZcoordinate a, XZcoordinate b) { return a.z < b.z; });
    XZcoordinate maxRight = *max_element(pointsSet.model_right.begin(), pointsSet.model_right.end(),
        [](XZcoordinate a, XZcoordinate b) { return a.z < b.z; });
    XZcoordinate minRight = *min_element(pointsSet.model_right.begin(), pointsSet.model_right.end(),
        [](XZcoordinate a, XZcoordinate b) { return a.z < b.z; });
    return MAXandMIN{ maxLeft.z, minLeft.z, maxRight.z, minRight.z };
}

void LiDAR2::printLeftRight(const NcmPoints& pointsSet) {
  std::cout << "left" << endl;
  for (auto& point : pointsSet.model_left) {
    std::cout << point.x << " " << point.z << endl;
  }
  std::cout << "right" << endl;
  for (auto& point : pointsSet.model_right) {
    std::cout << point.x << " " << point.z << endl;
  }
}

void StraightLine::renewRangebyXmax(float new_x_max)
{
  if (a > 0) {
    x_max = new_x_max;
    z_max = a * x_max + b;
  }
  else {
    x_max = new_x_max;
    z_min = a * x_max + b;
  }
}

void StraightLine::renewRangebyXmin(float new_x_min)
{
  if (a > 0) {
    x_min = new_x_min;
    z_min = a * x_min + b;
  }
  else {
    x_min = new_x_min;
    z_max = a * x_min + b;
  }
}

void StraightLine::renewRangebyZmax(float new_z_max)
{
  if (a > 0) {
    z_max = new_z_max;
    x_max = (z_max - b) / a;
  }
  else {
    z_max = new_z_max;
    x_min = (z_max - b) / a;
  }
}

void StraightLine::renewRangebyZmin(float new_z_min)
{
  if (a > 0) {
    z_min = new_z_min;
    x_min = (z_min - b) / a;
  }
  else {
    z_min = new_z_min;
    x_max = (z_min - b) / a;
  }
}

bool StraightLine::hasIntersection(const StraightLine& other) const
{
  if (a == other.a) {
    if (b == other.b) return true;
    return false;
  }
  if ((other.a * x_max + other.b < z_max && other.a * x_min + other.b > z_min) ||
    (a * other.x_max + b < other.z_max && a * other.x_min + b > other.z_min)) {
    return true;
  }
  return false;
}

XZcoordinate StraightLine::getIntersectionPoint(const StraightLine& other) const
{
  if (!hasIntersection(other)) {
    // 交差しない場合は (NaN, NaN) を返す
    return { std::nanf(""), std::nanf("") };
  }
  if (*this == other) {
    return { numeric_limits<float>::infinity(), numeric_limits<float>::infinity() };
  }

  // 交点を計算
  float x_intersection = (other.b - b) / (a - other.a);
  float z_intersection = a * x_intersection + b;
  return { x_intersection, z_intersection };
}