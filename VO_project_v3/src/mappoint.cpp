#include "zhou_vo/mappoint.hpp"

namespace zhou_vo{

	MapPoint::MapPoint() : id_(-1), p_w_(Eigen::Vector3d(0, 0, 0)), norm_(Eigen::Vector3d(0, 0, 0)),
						   good_(true), visible_times_(0), matched_times_(0) {

	}


	MapPoint::MapPoint(long id, Eigen::Vector3d p_w, Eigen::Vector3d norm, Frame* frame, const cv::Mat& descriptor) 
	: id_(id), p_w_(p_w), norm_(norm), good_(true), visible_times_(1), matched_times_(1), descriptor_(descriptor){
		observed_frames_.push_back(frame);
	}

	MapPoint::Ptr MapPoint::createMapPoint(){
		return MapPoint::Ptr(
				new MapPoint(factory_id_++, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)) );
	}

	MapPoint::Ptr MapPoint::createMapPoint(const Eigen::Vector3d& p_w, const Eigen::Vector3d& norm,
										   const cv::Mat& descriptor, Frame* frame){
		return MapPoint::Ptr(
				new MapPoint(factory_id_++, p_w, norm, frame, descriptor));
	}

	long MapPoint::factory_id_ = 0;
}