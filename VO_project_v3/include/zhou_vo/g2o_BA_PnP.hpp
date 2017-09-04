//Bundle adjustment by g2o for PnP, optimize the camera's pose se3
#ifndef G2O_BA_PNP_H
#define G2O_BA_PNP_H

#include "zhou_vo/common_include.hpp"
#include "zhou_vo/g2o_types.hpp"
#include "zhou_vo/frame.hpp"
#include "zhou_vo/mappoint.hpp"

/** 这里只优化相机位姿Ksi,通过最小化重投影误差来构建优化问题;
	自定义一个g2o中的优化边，只优化一个位姿，因此是一个一元边;
*/
namespace zhou_vo{

	void _3d2d_BA_g2o(	const std::vector<cv::Point3d> pts_3d,
						const std::vector<cv::Point2d> pts_2d,
						Frame::Ptr curr,
						cv::Mat& inliers_,
						std::vector<MapPoint::Ptr>& match_3dpts,
						Sophus::SE3& T_cw);
}
#endif