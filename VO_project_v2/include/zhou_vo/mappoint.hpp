#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "zhou_vo/common_include.hpp"

namespace zhou_vo{

	class MapPoint{
	public:
		typedef shared_ptr<MapPoint> Ptr;
		long id_; // ID
		Eigen::Vector3d p_w_; 	// position in world
		Eigen::Vector3d norm_; 	// normal of viewing direction
		cv::Mat descriptor_; 	// descriptor for matching
		int observed_times_; 	// being observed by feature matching algorithm
		int correct_times_; 	// being an inliner in pose estimation

		MapPoint();
		MapPoint(long id, Eigen::Vector3d p_w, Eigen::Vector3d norm);

		static MapPoint::Ptr createMapPoint();
	};
}

#endif