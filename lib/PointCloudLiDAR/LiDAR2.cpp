#include "LiDAR2.h"

NcmPoints LiDAR2::getNcmPoints(const LiDAR_degree& direction, float range) {
  // 中心の決定
  int center = 0;
  auto closest = pointCloud.begin();
  auto closest2 = pointCloud.end();
  switch (direction) { // もっと高速にはできるはずだよね、あとで考える
    case LiDAR_degree::FRONT:
      closest = min_element(pointCloud.begin(), pointCloud.begin()+128, [](XZcoordinate s1, XZcoordinate s2) {
        return (abs(s1.x) < abs(s2.x)) && s1.z > 0;
        });
      closest2 = min_element(pointCloud.begin()+384, pointCloud.end(), [](XZcoordinate s1, XZcoordinate s2) {
				return (abs(s1.x) < abs(s2.x)) && s1.z > 0;
				});
      if (abs(closest->x) > abs(closest2->x))  center = (int)std::distance(pointCloud.begin(), closest2);
			else center = (int)std::distance(pointCloud.begin(), closest);
		  break;
    case LiDAR_degree::BACK:
      closest = min_element(pointCloud.begin()+128, pointCloud.begin()+384, [](XZcoordinate s1, XZcoordinate s2) {
        return (abs(s1.x) < abs(s2.x)) && s1.z < 0;
        });

      center = (int)std::distance(pointCloud.begin(), closest);
		  break;
    case LiDAR_degree::LEFT:
      closest = min_element(pointCloud.begin()+256, pointCloud.end(), [](XZcoordinate s1, XZcoordinate s2) {
        return (abs(s1.z) < abs(s2.z)) && s1.x < 0;
        });

      center = (int)std::distance(pointCloud.begin(), closest);
      break;
    case LiDAR_degree::RIGHT:
      closest = min_element(pointCloud.begin(), pointCloud.begin()+256, [](XZcoordinate s1, XZcoordinate s2) {
        return (abs(s1.z) < abs(s2.z)) && s1.x > 0;
        });

      center = (int)std::distance(pointCloud.begin(), closest);
      break;
    default:
      break;
  }

  std::cout << "center: " << center << endl;
    
  // 中央10cmのデータを取得
  NcmPoints ncmP;
  float half_range = range / 2;

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
    else
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

WallSet LiDAR2::getWallType(const LiDAR_degree& direction)
{
  WallSet wallSet = {WallType::typeNo, WallType::center_n, WallType::typeNo};
    // 中央10cmのデータを取得
    NcmPoints pointsSet = getNcmPoints(direction, 10);
    rotateToFront(pointsSet, direction);
    MAXandMIN max_min = getMAX_MIN(pointsSet, direction);

    if (max_min.leftMax > 18.0F && max_min.rightMax > 18.0F) return wallSet;
    if (max_min.leftMin < 6.0F && max_min.rightMin < 6.0F) return { WallType::type10, WallType::center_n, WallType::type10 };
    return WallSet();
}

void LiDAR2::rotateToFront(NcmPoints& pointsSet, LiDAR_degree direction) {
  // 都合のいいように座標を回転
  switch (direction) {
  case LiDAR_degree::BACK:
    transform(pointsSet.model_left.begin(), pointsSet.model_left.end(), pointsSet.model_left.begin(),
      [](XZcoordinate element) {
        element.x = -1 * element.x;
        element.z = -1 * element.z;
        return element;
      });
    transform(pointsSet.model_right.begin(), pointsSet.model_right.end(), pointsSet.model_right.begin(),
      [](XZcoordinate element) {
        element.x = -1 * element.x;
        element.z = -1 * element.z;
        return element;
      });
    break;
  case LiDAR_degree::LEFT:
    transform(pointsSet.model_left.begin(), pointsSet.model_left.end(), pointsSet.model_left.begin(),
      [](XZcoordinate element) {
        float tmp = element.x;
        element.x = -1 * element.z;
        element.z = -1 * tmp;
        return element;
      });
    transform(pointsSet.model_right.begin(), pointsSet.model_right.end(), pointsSet.model_right.begin(),
      [](XZcoordinate element) {
        float tmp = element.x;
        element.x = -1 * element.z;
        element.z = -1 * tmp;
        return element;
      });
    break;
  case LiDAR_degree::RIGHT:
    transform(pointsSet.model_left.begin(), pointsSet.model_left.end(), pointsSet.model_left.begin(),
      [](XZcoordinate element) {
        float tmp = element.x;
        element.x = element.z;
        element.z = tmp;
        return element;
      });
    transform(pointsSet.model_right.begin(), pointsSet.model_right.end(), pointsSet.model_right.begin(),
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
