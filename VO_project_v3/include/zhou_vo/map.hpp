#ifndef MAP_H
#define MAP_H

#include "zhou_vo/common_include.hpp"
#include "zhou_vo/frame.hpp"
#include "zhou_vo/mappoint.hpp"

namespace zhou_vo{

	class Map{
	public:
		typedef shared_ptr<Map> Ptr;
		std::unordered_map<long, MapPoint::Ptr> map_points_; 	// all landmarks
		std::unordered_map<long, Frame::Ptr> keyframes_;		// all key frames

		Map(){}
		~Map(){}

		void insertKeyFrame(Frame::Ptr frame);
		void insertMapPoint(MapPoint::Ptr map_point);
	};
}

#endif // MAP_H