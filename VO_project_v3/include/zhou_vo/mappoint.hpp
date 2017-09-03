#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "zhou_vo/common_include.hpp"

namespace zhou_vo{

	class Frame;
	class MapPoint{
	public:
		typedef shared_ptr<MapPoint> Ptr;
		long 				id_; 				// ID
		static long 		factory_id_;		// factory id
		bool good_; 							// wheter a good point
		Eigen::Vector3d 	p_w_; 				// position in world
		Eigen::Vector3d 	norm_; 				// normal of viewing direction
		cv::Mat 			descriptor_; 		// descriptor for matching
		int 				matched_times_; 	// being an inlier in pose estimation
		int 				visible_times_; 	// being visible in current frame
		std::list<Frame*> 	observed_frames_; 	// key frames that can observe this point

		MapPoint();
		MapPoint(long id, Eigen::Vector3d p_w, Eigen::Vector3d norm, Frame* frame = NULL, const cv::Mat& descriptor = cv::Mat());

		inline cv::Point3d getPositionCV() const{
			return cv::Point3d(p_w_(0, 0), p_w_(1, 0), p_w_(2, 0));
		}

		static MapPoint::Ptr createMapPoint();
		static MapPoint::Ptr createMapPoint(const Eigen::Vector3d& p_w, const Eigen::Vector3d& norm,
											const cv::Mat& descriptor, Frame* frame);
	};
}

#endif