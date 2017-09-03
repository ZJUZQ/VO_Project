#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <algorithm>

#include "zhou_vo/config.hpp"
#include "zhou_vo/visual_odometry.hpp"

namespace zhou_vo{

	VisualOdometry::VisualOdometry() :
		state_(INITIALIZING), ref_(NULL), curr_(NULL), map_(new Map), num_lost_(0), num_inliers_(0) {

		num_features_ = Config::get<int> ("number_of_features");
		scale_factor_ = Config::get<double> ("scale_factor");
		level_pyramid_ = Config::get<int> ("level_pyramid");
		match_ratio_ = Config::get<double> ("match_ratio");
		max_num_lost_ = Config::get<int> ("max_num_lost");
		min_inliers_ = Config::get<int> ("min_inliers");
		key_frame_min_rotation_ = Config::get<double> ("key_frame_min_rotation");
		key_frame_min_translation_ = Config::get<double> ("key_frame_min_translation");
		orb_ = cv::ORB::create(num_features_, scale_factor_, level_pyramid_);
	}

	VisualOdometry::~VisualOdometry(){

	}

	bool VisualOdometry::addFrame(Frame::Ptr frame){
		switch(state_){

			case INITIALIZING:
				state_ = OK;
				curr_ = ref_ = frame;
				map_->insertKeyFrame(frame);
				extractKeyPoints();		// extract features from first frame
				computeDescriptors(); 
				setRef3DPoints();		// compute the 3d position of features in reference frame
				break;

			case OK:
				curr_ = frame;
				extractKeyPoints();
				computeDescriptors();
				featureMatching();
				poseEstimationPnP();
				if(checkEstimatedPose() == true){ 	// a good estimation
					curr_->T_cw_ = T_cr_ * ref_->T_cw_; 	// T_cw = T_cr * T_rw
					ref_ = curr_;
					setRef3DPoints();
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
		// match descriptors_ref and descriptors_curr, use OpenCV's brute force match
		std::vector<cv::DMatch> matches;
		cv::BFMatcher matcher(cv::NORM_HAMMING);
		matcher.match(descriptors_ref_, descriptors_curr_, matches);
		// select the best matches
		double min_dist = std::min_element( matches.begin(), matches.end(), 
											[] (const cv::DMatch& m1, const cv::DMatch& m2) { return m1.distance < m2.distance; } )->distance;
		feature_matches_.clear();
		std::vector<cv::DMatch>::iterator iter;
		for(iter = matches.begin(); iter != matches.end(); iter++){
			if(iter->distance < std::max<double>(min_dist * match_ratio_, 30.0))
				feature_matches_.push_back(*iter);
		}
		std::cout << "good matches: " << feature_matches_.size() << std::endl;
	}

	void VisualOdometry::poseEstimationPnP(){
		// construct the 3d-2d observations
		std::vector<cv::Point3d> pts3d;
		std::vector<cv::Point2d> pts2d;
		std::vector<cv::DMatch>::iterator iter;
		for(iter = feature_matches_.begin(); iter != feature_matches_.end(); iter++){
			pts3d.push_back(pts_3d_ref_[iter->queryIdx]);
			pts2d.push_back(kps_curr_[iter->trainIdx].pt);
		}
		cv::Mat K = (cv::Mat_<double>(3, 3) <<
					ref_->camera_->fx_, 0, ref_->camera_->cx_,
					0, ref_->camera_->fy_, ref_->camera_->cy_,
					0, 0, 1.0);
		cv::Mat rvec, tvec, inliers;
		cv::solvePnPRansac(pts3d, pts2d, K, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
		num_inliers_ = inliers.rows;
		std::cout << "PnP inliers: " << num_inliers_ << std::endl;
		T_cr_ = Sophus::SE3(
			Sophus::SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
			Eigen::Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));
	}

	void VisualOdometry::setRef3DPoints(){
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
	}

	void VisualOdometry::addKeyFrame(){
		std::cout << "adding a key frame" << std::endl;
		map_->insertKeyFrame(curr_);
	}

	bool VisualOdometry::checkEstimatedPose(){
		// check if the estimated pose is good
		if(num_inliers_ < min_inliers_){
			std::cout << "reject because inlier is too small: " << num_inliers_ << std::endl;
			return false;
		}
		// if the motion is too large, it is probably wrong
		Sophus::Vector6d se3 = T_cr_.log();
		if(se3.norm() > 5.0){
			std::cout << "reject because motion is too large, se3.norm = " << se3.norm() << std::endl;
			return false;
		}
		return true;
	}

	bool VisualOdometry::checkKeyFrame(){
		Sophus::Vector6d se3 = T_cr_.log();
		Eigen::Vector3d trans = se3.head<3>();
		Eigen::Vector3d rot = se3.tail<3>();
		if(rot.norm() > key_frame_min_rotation_ || trans.norm() > key_frame_min_translation_)
			return true;
		return false;
	}
}