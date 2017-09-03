#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// define the commonly included file to avoid a long include list

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sophus/se3.h>
#include <sophus/so3.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <unordered_map>
#include <memory>
using namespace std;

#endif // COMMON_INCLUDE_H