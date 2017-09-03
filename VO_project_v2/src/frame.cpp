#include "zhou_vo/frame.hpp"

namespace zhou_vo{
	Frame::Frame() : id_(-1), time_stamp_(-1), camera_(NULL){

	}

	Frame::Ptr Frame::createFrame(){
		static long factory_id = 0;
		return Frame::Ptr(new Frame(factory_id++));
	}

	// find the depth
	double Frame::findDepth(const cv::KeyPoint& kp) const{
		int x = cvRound(kp.pt.x);
		int y = cvRound(kp.pt.y);
		unsigned short d = depth_.ptr<unsigned short>(y)[x];
		if(d != 0)
			return double(d) / camera_->depth_scale_;
		else{
			// check the nearby points
			int dx[4] = {-1, 0, 1, 0};
			int dy[4] = {0, -1, 0, 1};
			for(int i = 0; i < 4; i++){
				d = depth_.ptr<unsigned short>(y + dy[i])[x + dx[i]];
				if(d != 0)
					return double(d) / camera_->depth_scale_;
			}
		}
		return -1.0;
	}

	Eigen::Vector3d Frame::getCameraCenter() const{
		return T_cw_.inverse().translation();
	}

	bool Frame::isInFrame(const Eigen::Vector3d& p_w) const{
		Eigen::Vector3d p_c = camera_->world2camera(p_w, T_cw_);
		if(p_c(2, 0) < 0)
			return false;
		Eigen::Vector2d p_p = camera_->world2pixel(p_w, T_cw_);
		return (p_p(0, 0) > 0 && p_p(0, 0) + 1 < color_.cols && p_p(1, 0) > 0 && p_p(1, 0) + 1 < color_.rows);
	}
}