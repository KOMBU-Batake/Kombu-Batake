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
      if (abs(readPoint(i).z - centralZ) <= half_range && abs(readPoint(i).x) > 5) leftStart = i;
    }
    ncmP.count_left = center - leftStart;
    ncmP.model_left.reserve(ncmP.count_left);
    for (int i = leftStart; i <= center; i++) {
			ncmP.model_left.push_back(readPoint(i));
		}
    // 右方向
    int rightEnd = center + 1;
    for (int i = center; i < center + 80; i++) {
      if (abs(readPoint(i).z - centralZ) <= half_range && abs(readPoint(i).x) > 5) rightEnd = i;
    }
    ncmP.count_right = rightEnd - center;
    ncmP.model_right.reserve(ncmP.count_right);
    for (int i = center + 1; i <= rightEnd; i++) {
			ncmP.model_right.push_back(readPoint(i));
		}
    //std::cout << "leftStart: " << leftStart << " rightEnd: " << rightEnd << endl;
  }
  else { // FTONT or BACK
    float centralX = pointCloud[center].x;
    // 左方向
    int leftStart = center;
    for (int i = center; i > center - 80; i--) {
      if (abs(readPoint(i).x - centralX) <= half_range && abs(readPoint(i).z) > 5) leftStart = i;
    }
    ncmP.count_left = center - leftStart;
    ncmP.model_left.reserve(ncmP.count_left);
    for (int i = leftStart; i <= center; i++) {
      ncmP.model_left.push_back(readPoint(i));
    }
    // 右方向
    int rightEnd = center + 1;
    for (int i = center + 1; i < center + 80; i++) {
      if (abs(readPoint(i).x - centralX) <= half_range && abs(readPoint(i).z) > 5) rightEnd = i;
    }
    ncmP.count_right = rightEnd - center;
    ncmP.model_right.reserve(ncmP.count_right);
    for (int i = center + 1; i <= rightEnd; i++) {
			ncmP.model_right.push_back(readPoint(i));
		}
    //std::cout << "leftStart: " << leftStart << " rightEnd: " << rightEnd << endl;
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
    frontCenterNum = center;
    break;
  case LiDAR_degree::BACK:
    closest = min_element(pointCloud.begin() + 128, pointCloud.begin() + 384, [](XZcoordinate s1, XZcoordinate s2) {
      return (abs(s1.x) < abs(s2.x)) && s1.z < 0;
      });

    center = (int)std::distance(pointCloud.begin(), closest);
    backCenterNum = center;
    break;
  case LiDAR_degree::LEFT:
    closest = min_element(pointCloud.begin() + 256, pointCloud.end(), [](XZcoordinate s1, XZcoordinate s2) {
      return (abs(s1.z) < abs(s2.z)) && s1.x < 0;
      });

    center = (int)std::distance(pointCloud.begin(), closest);
    leftCenterNum = center;
    break;
  case LiDAR_degree::RIGHT:
    closest = min_element(pointCloud.begin(), pointCloud.begin() + 256, [](XZcoordinate s1, XZcoordinate s2) {
      return (abs(s1.z) < abs(s2.z)) && s1.x > 0;
      });

    center = (int)std::distance(pointCloud.begin(), closest);
    rightCenterNum = center;
    break;
  default:
    break;
  }
  return center;
}

vector<int> LiDAR2::getNcmNumbers(const LiDAR_degree& direction, float range) {
  // 中央10cmのデータを取得
  int center = getCenterNum(direction);
  float half_range = range / 2;
  int start = center - 100;
  int end = center + 100;
  vector<int> ncmNumbers;
  if (direction == LiDAR_degree::FRONT || direction == LiDAR_degree::BACK) {
    for (int i = start; i <= end; i++) if (abs(readPoint(i).x) <= half_range) {
      ncmNumbers.push_back(i);
    }
  }
  else { // LEFT or RIGHT
    for (int i = start; i <= end; i++) if (abs(readPoint(i).z) <= half_range) ncmNumbers.push_back(i);
  }
  return ncmNumbers;
}

float LiDAR2::Variance2(vector<XZcoordinate>& hoge,int size) {
  vector<XZcoordinate>fuga(size);
  for (size_t i = 0; i < hoge.size(); ++i) {
    fuga[i].z = hoge[i].x * hoge[i].x + hoge[i].z * hoge[i].z;
  }
  return Variance(fuga.begin(), fuga.end(), 1, size);
}

WallSet LiDAR2::getWallType(const LiDAR_degree& direction)
{
  WallSet wallSet = {WallType::typeNo, WallType::center_n, WallType::typeNo};
    // 中央10cmのデータを取得
    NcmPoints pointsSet = getNcmPoints(direction, 13);

    //std::cout << "-------------------" << endl;
    //printLeftRight(pointsSet);
    //std::cout << "-------------------" << endl;

    rotateToFront(pointsSet.model_left, direction);
    rotateToFront(pointsSet.model_right, direction);

    bool isLeftEmpty = pointsSet.isLeftEmpty();
    bool isRightEmpty = pointsSet.isRightEmpty();
    if (pointsSet.leftClosest < 6) {
      float var = Variance2(pointsSet.model_left, (int)pointsSet.model_left.size());
      if (var < 1) wallSet.left = WallType::type16;
      //cout << "variance L : " << var << endl;
    }
    if (pointsSet.rightClosest < 6) {
			float var = Variance2(pointsSet.model_right, (int)pointsSet.model_right.size());
      if (var < 1) wallSet.right = WallType::type17;
      //cout << "variance R : " << var << endl;
		}
    vector<int> featurePoints;
    if (wallSet.left != WallType::type16 && wallSet.right != WallType::type17) featurePoints = VectorTracer(pointsSet);

    if (! (isLeftEmpty || wallSet.left == WallType::type16)) wallSet.left = identifyLeft(pointsSet, featurePoints);
    //std::cout << "* left: " << (int)wallSet.left << endl;
    if (! (isRightEmpty || wallSet.right == WallType::type17)) wallSet.right = identifyRight(pointsSet, featurePoints);
    //std::cout << "* right: " << (int)wallSet.right << endl;
    wallSet.center = identifyCenter(pointsSet, wallSet, featurePoints);
    //cout << "* center: " << (int)wallSet.center << endl;

    return wallSet;
}

WallType LiDAR2::identifyLeft(NcmPoints& pointSet, vector<int>& featurePoints)
{
  // 範囲外をカット
  int start = 0;
  for (int i = 0; i < pointSet.model_left.size(); i++) {
    if (pointSet.model_left[i].x < -6) {
			start = i+1;
		}
	}
  // 左側の壁をカット
  XZrange rangeToCut = {-4,-6.1f,18,0};
  for (auto& num: featurePoints) {
    if (rangeToCut.isIncluding(pointSet.read(num)) && num > start) {
			start = num;
		}
	}
  //std::cout << "L start is: " << start;
  start += 2;

  // 右側の壁をカット
  XZrange rangeToCut2 = { 0.1f, -2, 18, 0 };
  int end = pointSet.count_left;
  for (auto& num : featurePoints) {
    if (num > pointSet.count_left) break;
    if (rangeToCut2.isIncluding(pointSet.read(num))) {
			end = num;
      break;
		}
	}
  //std::cout << ", L end is: " << end << endl;
  end -= 2;

  if (start >= end) return WallType::typeNo;

  // Z軸方向の最大値と最小値を取得
  auto startIt = pointSet.model_left.begin() + start;
  auto endIt = pointSet.model_left.begin() + end;
  float maxLeft = max_element(startIt, endIt, [](XZcoordinate a, XZcoordinate b) { return a.z < b.z; })->z;
  //float minLeft = min_element(startIt, endIt, [](XZcoordinate a, XZcoordinate b) { return a.z < b.z; })->z;
  if (maxLeft > 18) return WallType::typeNo; // 壁がないと判断
  // cout << "max: " << maxLeft << " min: " << minLeft << endl;

  float variance = Variance(startIt, endIt, start, end);

  if (variance < 0.1f) { // まっすぐな壁
    if (maxLeft > 15) return WallType::type0;
    else if (maxLeft > 10) return WallType::type5;
    else return WallType::type10;
  }

  // 曲線
  return identifyCurve(startIt, endIt, pointSet.leftClosest);
}

WallType LiDAR2::identifyRight(NcmPoints& pointSet, vector<int>& featurePoints)
{
  // 範囲外をカット
  int end_tmp = pointSet.count_left;
  for (size_t i = 0; i < pointSet.model_right.size(); i++) {
    if (pointSet.model_right[i].x > 6) {
      //cout << "RR cut" << endl;
			break;
    }
    else {
			end_tmp++;
    }
	}
  // 右側の壁をカット
  XZrange rangeToCut = { 4, 6.1f, 18, 0 };
  int end = pointSet.count_left + pointSet.count_right + 1;
  for (auto& num : featurePoints) {
    if (rangeToCut.isIncluding(pointSet.read(num))) {
			end = num;
			break;
		}
	}
  if (end > end_tmp) end = end_tmp;
  //std::cout << "R end is: " << end;
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
  //std::cout << ", R start is: " << start << endl;
  start += 2;
  start -= pointSet.count_left;

  if (start >= end ) return WallType::typeNo;

  // Z軸方向の最大値と最小値を取得
  auto startIt = pointSet.model_right.begin() + start;
  auto endIt = pointSet.model_right.begin() + end;
  float maxRight = max_element(startIt, endIt, [](XZcoordinate a, XZcoordinate b) { return a.z < b.z; })->z;
  //float minRight = min_element(startIt, endIt, [](XZcoordinate a, XZcoordinate b) { return a.z < b.z; })->z;
  if (maxRight > 18) return WallType::typeNo; // 壁がないと判断
  // cout << "max: " << maxRight << " min: " << minRight << endl;

  float variance= Variance(startIt, endIt, start, end);

  if (variance < 0.1f) { // まっすぐな壁
  	if (maxRight > 15) return WallType::type0;
  	else if (maxRight > 10) return WallType::type5;
  	else return WallType::type10;
  }

  // 曲線
  return identifyCurve(startIt, endIt, pointSet.rightClosest);
}

WallType LiDAR2::identifyCenter(NcmPoints& pointSet, const WallSet& wallset, vector<int>& featurePoints)
{
  if (wallset.left == WallType::type16 || wallset.right == WallType::type17) return WallType::center_n;
  unordered_set<uint8_t> left_back = {11u,0u,2u,4u};
  unordered_set<uint8_t> right_back = {11u,0u,1u,3u};
  unordered_set<uint8_t> left_front = {5u,7u,9u};
  unordered_set<uint8_t> right_front = {5u,6u,8u};
  float centerZ = pointSet.model_left[pointSet.count_left].z;
  //cout << "centerZ: " << centerZ << endl;
  
  // 中央奥
  if (left_back.count((uint8_t)wallset.left) && right_back.count((uint8_t)wallset.right)) { 
    if (centerZ >= 5.1f && centerZ <= 6.6f) return WallType::center_s;
    else if (centerZ >= 11.1f && centerZ <= 12.9f) return WallType::center_o;
    else return WallType::center_n;
	}
  
  // 中央手前
  bool c_left_front = left_front.count((uint8_t)wallset.left);
  bool c_right_front = right_front.count((uint8_t)wallset.right);
  if ((c_left_front && c_right_front) ||
      (c_left_front && (uint8_t)wallset.right < 5) ||
      (c_right_front && (uint8_t)wallset.left < 5)) {
    if (centerZ >= 5.1f && centerZ <= 6.9f) return WallType::center_s;
    else return WallType::center_n;
  }

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

vector<int> LiDAR2::VectorTracer(NcmPoints& pointSet, bool display)
{
  //std::cout << "-------------------" << endl;
	vector<vector<int>> featurePointsNum;
  vector<vector<float>> featurePointsTheta;
  float theta = 0, abstheta = 0;
  XZcoordinate p1, p2, p3;
  int start = pointSet.centerNum - pointSet.count_left;
  int last_updated_i = 0;
  float last_angle = 0;
  for (int i = start; i <= pointSet.centerNum + pointSet.count_right; i++) {
    p1 = (readPoint(i - 1) + readPoint(i - 2)) / 2;
    p2 = readPoint(i);
    p3 = (readPoint(i + 1) + readPoint(i + 2)) / 2;
    theta = calculateAngle(p1, p2, p3);
    abstheta = abs(theta);
    if (abstheta <= 360 && abstheta > 225 || abstheta >= 0 && abstheta < 135) {
      if (i == last_updated_i + 1 && (abs(last_angle - getAngle(p1, p2, p3)) < 45 || abs(last_angle - getAngle(p1, p2, p3)) > 315)) {
        featurePointsNum[featurePointsNum.size()-1].push_back(i - start);
        featurePointsTheta[featurePointsTheta.size()-1].push_back(theta);
        //cout << "* " << i - start << " " << theta << " " << getAngle(p1, p2, p3) << endl;
      }
      else {
				featurePointsNum.push_back({i - start});
        featurePointsTheta.push_back({ theta });
        //cout << "~~~~~~~~~" << endl << "* " << i - start << " " << theta << " " << getAngle(p1, p2, p3) << endl;
			}
      
      last_updated_i = i;
      last_angle = getAngle(p1, p2, p3);
    }
  //  else {
		//	cout << "n " << i - start << " " << theta << " " << getAngle(p1, p2, p3) << endl;
		//}
	}

  vector<int> returnNum;
  for (size_t i = 0, size = featurePointsNum.size(); i < size; i++) {
    int minDistance = (int)std::distance(featurePointsTheta[i].begin(), min_element(featurePointsTheta[i].begin(), featurePointsTheta[i].end()));
    returnNum.push_back(featurePointsNum[i][minDistance]);
	}

  if (display) {
    std::cout << "feature points -------------------" << endl;
    for (auto& num : returnNum) {
      std::cout << num + start << " " << num << endl;
    }
  }
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
        element.x = element.z;
        element.z = -1 * tmp;
        return element;
      });
    break;
  case LiDAR_degree::RIGHT:
    transform(points.begin(), points.end(), points.begin(),
      [](XZcoordinate element) {
        float tmp = element.x;
        element.x = -1 * element.z;
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
        std::cout << "num: " << index << ", " <<  "cond: " << (int)condition << ", " << line.x_min << " " << line.x_max << " " << line.z_min << " " << line.z_max;
        if (condition &&
           (direction == LiDAR_degree::FRONT && line.z_max >= center.z ||
            direction == LiDAR_degree::BACK && line.z_min <= center.z)) {
          std::cout << " push";
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
        std::cout << endl;
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
          std::cout << index << ", " << (int)condition;
        }
        index++;
      }
      break;
  }
  return lines;
}

vector<ImageXZcoordinate> LiDAR2::getPositionOfImage(vector<int>& numbers) {
  vector<float> x_range_start(37), z_range_start(41), x_range_end(37), z_range_end(41);
  for (int i = 0; i < 37; i++) {
    x_range_start[i] = -6 + (12.0f / 37.0f) * i;
    x_range_end[i] = -6 + (12.0f / 37.0f) * (i + 1);
  }
  for (int i = 0; i < 41; i++) {
    z_range_start[i] = 18 - (12.0f / 37.0f) * i;
    z_range_end[i] = 18 - (12.0f / 37.0f) * (i + 1);
  }

  // x軸方向
  vector<int> optimizedNumbers;
  vector<ImageXZcoordinate> optimizedPoints;
  int last_num = -1;
  for (auto& number : numbers) {
    for (int i = 0; i < 37; i++) {
      if (readPoint(number).x >= x_range_start[i] && readPoint(number).x <= x_range_end[i] && i != last_num) {
        int z = convertLiADRtoImage(readPoint(number).z, z_range_start, z_range_end);
        if (z == -1) continue;
        optimizedNumbers.push_back(number);
        optimizedPoints.push_back({ i + 24, z });
        //cout << number << ": " << i + 24 << "," << z << endl;
        last_num = i;
        break;
      }
    }
  }
  return optimizedPoints;
}

canGo LiDAR2::isClear(const LiDAR_degree& direction) {
  XZcoordinate center = {0,0};
  int start = 0, end = 0;
  switch (direction) {
    case LiDAR_degree::FRONT:
      center = { 0, 6 };
      start = frontCenterNum-128;
      end = frontCenterNum+ 128;
      break;
    case LiDAR_degree::LEFT:
      center = { -6, 0 };
      start = leftCenterNum - 128;
      end = leftCenterNum + 128;
      break;
    case LiDAR_degree::BACK:
      center = { 0, -6 };
      start = backCenterNum - 128;
      end = backCenterNum + 128;
      break;
    case LiDAR_degree::RIGHT:
      center = { 6, 0 };
      start = rightCenterNum - 128;
      end = rightCenterNum + 128;
      break;
    case LiDAR_degree::FRONT_LEFT:
			center = { -6, 6 };
      start = leftCenterNum;
      end = frontCenterNum;
      if (start > end) end += 512;
			break;
    case LiDAR_degree::FRONT_RIGHT:
      center = { 6, 6 };
      start = frontCenterNum;
			end = rightCenterNum;
      if (start > end) start -= 512;
      break;
  }
  bool ok = true;
  for (int i = start; i <= end; i++) {
    XZcoordinate p = readPoint(i);
    if (pow(p.x - center.x, 2) + pow(p.z - center.z, 2) < 16) {
      ok = false;
			break;
    }
  }
  if (ok) return canGo::GO;

  float range = 1;

  switch (direction) {
  case LiDAR_degree::FRONT:
    center = { -1*range, 6 };
    break;
  case LiDAR_degree::LEFT:
    center = { -6, -1*range };;
    break;
  case LiDAR_degree::BACK:
    center = { range, -6 };
    break;
  case LiDAR_degree::RIGHT:
    center = { 6, range };
    break;
  }
  ok = true;
  for (int i = start; i <= end; i++) {
    XZcoordinate p = readPoint(i);
    if (pow(p.x - center.x, 2) + pow(p.z - center.z, 2) < 16) {
      ok = false;
      break;
    }
  }
  if (ok) return canGo::GO_leftO;

  switch (direction) {
  case LiDAR_degree::FRONT:
    center = { range, 6 };
    break;
  case LiDAR_degree::LEFT:
    center = { -6, range };;
    break;
  case LiDAR_degree::BACK:
    center = { -1*range, -6 };
    break;
  case LiDAR_degree::RIGHT:
    center = { 6, -1* range };
    break;
  }

  for (int i = start; i <= end; i++) {
    XZcoordinate p = readPoint(i);
    if (pow(p.x - center.x, 2) + pow(p.z - center.z, 2) < 16) return canGo::NO;
  }
  return canGo::GO_rightO;
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