//Bundle adjustment by Ceres for PnP, optimize the camera's pose se3
#ifndef CERES_BA_PNP_H
#define CERES_BA_PNP_H

#include "zhou_vo/common_include.hpp"
#include "zhou_vo/mappoint.hpp"

namespace zhou_vo{

	void _3d2d_BA_ceres(	const std::vector<cv::Point3d> pts_3d,
							const std::vector<cv::Point2d> pts_2d,
							const cv::Mat& K,
							cv::Mat& inliers_,
							std::vector<MapPoint::Ptr>& match_3dpts,
							Sophus::SE3& T_cw);
}
#endif