#pragma once
#include "PointCloudLiDAR.h"

struct NcmPack {
  vector<float> left;
  vector<float> right;
};

class LiDAR2 :
    public PointCloudLiDAR
{
public:
  NcmPack getNcm() {

  }
};

