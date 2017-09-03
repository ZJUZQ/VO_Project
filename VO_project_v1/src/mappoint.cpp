#include "zhou_vo/mappoint.hpp"

namespace zhou_vo{

	MapPoint::MapPoint() : id_(-1), p_w_(Eigen::Vector3d(0, 0, 0)), norm_(Eigen::Vector3d(0, 0, 0)),
						   observed_times_(0), correct_times_(0) {

	}

	MapPoint::MapPoint(long id, Eigen::Vector3d p_w, Eigen::Vector3d norm)
		: id_(id), p_w_(p_w), norm_(norm), observed_times_(0), correct_times_(0){

	}

	MapPoint::Ptr MapPoint::createMapPoint(){
		static long factory_id = 0;
		return MapPoint::Ptr(
				new MapPoint(factory_id++, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)) );
	}
}