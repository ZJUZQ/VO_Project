#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "zhou_vo/common_include.hpp"
#include "zhou_vo/map.hpp"

#include "opencv2/features2d/features2d.hpp"

#include "zhou_vo/Ceres_BA_PnP.hpp"
#include "zhou_vo/g2o_BA_PnP.hpp"

namespace zhou_vo{

	class VisualOdometry{
	public:
		typedef std::shared_ptr<VisualOdometry> Ptr;
		enum VOState{
			INITIALIZING = -1,
			OK = 0,
			LOST
		};

		VOState 	state_;		// current VO status
		Map::Ptr 	map_; 		// map with all frames and map points
		Frame::Ptr 	ref_;		// reference frame
		Frame::Ptr 	curr_; 		// current frame

		cv::Ptr<cv::ORB> orb_;  // orb detector and computer 
    	std::vector<cv::KeyPoint>    kps_curr_;    		// keypoints in current frame
    	cv::Mat                     descriptors_curr_;  // descriptor in current frame 
    
    	cv::FlannBasedMatcher   matcher_flann_;     	// flann matcher
    	std::vector<MapPoint::Ptr>   match_3dpts_;      // matched 3d points 
    	std::vector<int>             match_2dkp_index_; // matched 2d pixels (index of kp_curr)

		Sophus::SE3 T_cw_estimated_;    // the estimated pose of current frame 
    	int num_inliers_;        // number of inlier features in icp
    	int num_lost_;           // number of lost times

		// parameters
		int 	num_features_; 		// number of features
		double	scale_factor_;		// scale in image pyramid
		int 	level_pyramid_;		// number of pyramid levels
		double 	match_ratio_;		// ratio for selecting good matches
		int 	max_num_lost_; 		// max number of continuous lost times
		int 	min_inliers_; 		// minimum inliers

		double 	key_frame_min_rotation_; 	// minimum rotation of two key frames
		double 	key_frame_min_translation_;	// minimum translation of two key frames
		double  map_point_erase_ratio_; 	// remove map point ratio

	public:
		VisualOdometry();
		~VisualOdometry();
		bool addFrame(Frame::Ptr frame);

	protected:
		// inner operation
		void extractKeyPoints();
		void computeDescriptors();
		void featureMatching();
		void poseEstimationPnP(); 
		void setRef3DPoints();
		void optimizeMap();

		void addKeyFrame();
		void addMapPoints();
		bool checkEstimatedPose();
		bool checkKeyFrame();

		double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );

	};
}

#endif