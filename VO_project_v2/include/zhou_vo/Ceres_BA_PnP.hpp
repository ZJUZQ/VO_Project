//Bundle adjustment by Ceres for PnP, optimize the camera's pose se3
#ifndef CERES_BA_PNP_H
#define CERES_BA_PNP_H

#include "zhou_vo/common_include.hpp"

void bundle_adjustment_ceres(const std::vector<cv::Point3d> pts_3d,
							 const std::vector<cv::Point2d> pts_2d,
							 const cv::Mat& K,
							 Sophus::SE3& T_cr);

#endif