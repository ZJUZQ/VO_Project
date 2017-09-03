#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <algorithm>

#include "zhou_vo/config.hpp"
#include "zhou_vo/visual_odometry.hpp"

namespace zhou_vo{

	VisualOdometry::VisualOdometry() :
		state_(INITIALIZING), ref_(NULL), curr_(NULL), map_(new Map), num_lost_(0), num_inliers_(0), matcher_flann_ ( new cv::flann::LshIndexParams ( 5,10,2 )) {

		num_features_ = Config::get<int> ("number_of_features");
		scale_factor_ = Config::get<double> ("scale_factor");
		level_pyramid_ = Config::get<int> ("level_pyramid");
		match_ratio_ = Config::get<double> ("match_ratio");
		max_num_lost_ = Config::get<int> ("max_num_lost");
		min_inliers_ = Config::get<int> ("min_inliers");
		key_frame_min_rotation_ = Config::get<double> ("key_frame_min_rotation");
		key_frame_min_translation_ = Config::get<double> ("key_frame_min_translation");
		map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );
		orb_ = cv::ORB::create(num_features_, scale_factor_, level_pyramid_);
	}

	VisualOdometry::~VisualOdometry(){

	}

	bool VisualOdometry::addFrame(Frame::Ptr frame){
		switch(state_){

			case INITIALIZING:
				state_ = OK;
				curr_ = ref_ = frame;
				// extract features from first frame and add them into map
		        extractKeyPoints();
		        computeDescriptors();
		        addKeyFrame();      // the first frame is a key-frame
				break;

			case OK:
				curr_ = frame;
				curr_->T_cw_ = ref_->T_cw_;
				extractKeyPoints();
				computeDescriptors();
				featureMatching();
				poseEstimationPnP();
				if(checkEstimatedPose() == true){ 	// a good estimation
					curr_->T_cw_ = T_cw_estimated_;
            		optimizeMap();
					num_lost_ = 0;
					if(checkKeyFrame() == true) // is a key frame
						addKeyFrame();
				}
				else{	// a bad estimation due to various reasons
					num_lost_++;
					if(num_lost_ > max_num_lost_)
						state_ = LOST;
					return false;
				}
				break;

			case LOST:
				std::cout << "vo has lost." << std::endl;
				break;
		}
		return true;
	}

	void VisualOdometry::extractKeyPoints(){
		orb_->detect(curr_->color_, kps_curr_);
	}

	void VisualOdometry::computeDescriptors(){
		orb_->compute(curr_->color_, kps_curr_, descriptors_curr_);
	}

	void VisualOdometry::featureMatching(){
		
		std::vector<cv::DMatch> matches;
		// select the candidates in map 
	    cv::Mat desp_map;
	    std::vector<MapPoint::Ptr> candidate;
	    for ( auto& allpoints: map_->map_points_ )
	    {
	        MapPoint::Ptr& p = allpoints.second;
	        // check if p in curr frame image 
	        if ( curr_->isInFrame(p->p_w_) )
	        {
	            // add to candidate 
	            p->visible_times_++;
	            candidate.push_back( p );
	            desp_map.push_back( p->descriptor_ );
	        }
	    }
	     matcher_flann_.match ( desp_map, descriptors_curr_, matches );

		// select the best matches
		double min_dist = std::min_element( matches.begin(), matches.end(), 		// lambda表达式
											[] (const cv::DMatch& m1, const cv::DMatch& m2) { return m1.distance < m2.distance; } )->distance; 
		
		match_3dpts_.clear();
    	match_2dkp_index_.clear();
		std::vector<cv::DMatch>::iterator iter;

		for(iter = matches.begin(); iter != matches.end(); iter++){
			if(iter->distance < std::max<double>(min_dist * match_ratio_, 30.0)){
				match_3dpts_.push_back( candidate[iter->queryIdx] );
            	match_2dkp_index_.push_back( iter->trainIdx );
			}
		}
		std::cout << "good matches: " << match_3dpts_.size() << std::endl;
	}

	void VisualOdometry::poseEstimationPnP(){
		// construct the 3d-2d observations
		std::vector<cv::Point3d> pts3d;
		std::vector<cv::Point2d> pts2d;
		for ( int index:match_2dkp_index_ ){
        	pts2d.push_back ( kps_curr_[index].pt );
    	}
		for ( MapPoint::Ptr pt:match_3dpts_ ){
        	pts3d.push_back( pt->getPositionCV() );
    	}

		cv::Mat K = (cv::Mat_<double>(3, 3) <<
					ref_->camera_->fx_, 0, ref_->camera_->cx_,
					0, ref_->camera_->fy_, ref_->camera_->cy_,
					0, 0, 1.0);
		cv::Mat rvec, tvec, inliers;
		cv::solvePnPRansac(pts3d, pts2d, K, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
		num_inliers_ = inliers.rows;
		std::cout << "PnP inliers: " << num_inliers_ << std::endl;
		T_cw_estimated_ = Sophus::SE3(
			Sophus::SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
			Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));

		std::cout << "T_cw_estimated_: \n" << T_cw_estimated_.matrix() << std::endl;

		// using bundle adjustment to optimize the camera pose with ceres
		bundle_adjustment_ceres(pts3d, pts2d, K, T_cw_estimated_);

		std::cout << "T_cw_estimated_: \n" << T_cw_estimated_.matrix() << std::endl;
	}

	/*void VisualOdometry::setRef3DPoints(){
		// select the features with depth measurements
		pts_3d_ref_.clear();
		descriptors_ref_ = cv::Mat();
		for(size_t i = 0; i < kps_curr_.size(); i++){
			// size_t is the type returned by the sizeof operator and is widely used in the standard library to represent sizes and counts.
			double d = ref_->findDepth(kps_curr_[i]);
			if(d > 0){
				Eigen::Vector3d p_cam = ref_->camera_->pixel2camera(
					Eigen::Vector2d(kps_curr_[i].pt.x, kps_curr_[i].pt.y), d);
				pts_3d_ref_.push_back(cv::Point3d(p_cam(0, 0), p_cam(1, 0), p_cam(2, 0)));
				descriptors_ref_.push_back(descriptors_curr_.row(i));
			}

		}
	}*/

	void VisualOdometry::addMapPoints()
	{
	    // add the new map points into map
	    std::vector<bool> matched(kps_curr_.size(), false); 
	    for ( int index:match_2dkp_index_ )
	        matched[index] = true;
	    for ( int i = 0; i < kps_curr_.size(); i++ ){
	        if ( matched[i] == true )   
	            continue;
	        double d = ref_->findDepth ( kps_curr_[i] ); // new point
	        if ( d < 0 )  
	            continue;
	        Eigen::Vector3d p_world = ref_->camera_->pixel2world (
	            Eigen::Vector2d ( kps_curr_[i].pt.x, kps_curr_[i].pt.y ), 
	            curr_->T_cw_, d
	        );
	        Eigen::Vector3d n = p_world - ref_->getCameraCenter();
	        n.normalize();
	        MapPoint::Ptr map_point = MapPoint::createMapPoint(
	            p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
	        );
	        map_->insertMapPoint( map_point );
	    }
	}

	void VisualOdometry::optimizeMap()
	{
	    // remove the hardly seen and no visible points 
	    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
	    {
	        if ( !curr_->isInFrame(iter->second->p_w_) )
	        {
	            iter = map_->map_points_.erase(iter);
	            continue;
	        }
	        float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
	        if ( match_ratio < map_point_erase_ratio_ )
	        {
	            iter = map_->map_points_.erase(iter);
	            continue;
	        }
	        
	        double angle = getViewAngle( curr_, iter->second );
	        if ( angle > M_PI/6. )
	        {
	            iter = map_->map_points_.erase(iter);
	            continue;
	        }
	        if ( iter->second->good_ == false )
	        {
	            // TODO try triangulate this map point 
	        }
	        iter++;
	    }
	    
	    if ( match_2dkp_index_.size()<100 )
	        addMapPoints();
	    if ( map_->map_points_.size() > 1000 )  
	    {
	        // TODO map is too large, remove some one 
	        map_point_erase_ratio_ += 0.05;
	    }
	    else 
	        map_point_erase_ratio_ = 0.1;
	    cout<<"map points: "<<map_->map_points_.size()<<endl;
	}

	double VisualOdometry::getViewAngle ( Frame::Ptr frame, MapPoint::Ptr point )
	{
	    Eigen::Vector3d n = point->p_w_ - frame->getCameraCenter();
	    n.normalize();
	    return acos( n.transpose()*point->norm_ );
	}

	void VisualOdometry::addKeyFrame(){
		std::cout << "adding a key frame" << std::endl;
		if ( map_->keyframes_.empty() ){
	        // first key-frame, add all 3d points into map
	        for ( size_t i = 0; i < kps_curr_.size(); i++ ){
	            double d = curr_->findDepth ( kps_curr_[i] );
	            if ( d < 0 ) 
	                continue;
	            Eigen::Vector3d p_world = ref_->camera_->pixel2world (
	                Eigen::Vector2d ( kps_curr_[i].pt.x, kps_curr_[i].pt.y ), curr_->T_cw_, d
	            );
	            Eigen::Vector3d n = p_world - ref_->getCameraCenter();
	            n.normalize();
	            MapPoint::Ptr map_point = MapPoint::createMapPoint(
	                p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
	            );
	            map_->insertMapPoint( map_point );
	        }
	    }
		map_->insertKeyFrame(curr_);
		ref_ = curr_;
	}

	bool VisualOdometry::checkEstimatedPose(){
		// check if the estimated pose is good
		if(num_inliers_ < min_inliers_){
			std::cout << "reject because inlier is too small: " << num_inliers_ << std::endl;
			return false;
		}
		// if the motion is too large, it is probably wrong
		Sophus::SE3 T_rc = ref_->T_cw_ * T_cw_estimated_.inverse();
		Sophus::Vector6d se3 = T_rc.log();
		if(se3.norm() > 5.0){
			std::cout << "reject because motion is too large, se3.norm = " << se3.norm() << std::endl;
			return false;
		}
		return true;
	}

	bool VisualOdometry::checkKeyFrame(){
		Sophus::SE3 T_rc = ref_->T_cw_ * T_cw_estimated_.inverse();
		Sophus::Vector6d se3 = T_rc.log();
		Eigen::Vector3d trans = se3.head<3>();
		Eigen::Vector3d rot = se3.tail<3>();
		if(rot.norm() > key_frame_min_rotation_ || trans.norm() > key_frame_min_translation_)
			return true;
		return false;
	}
}