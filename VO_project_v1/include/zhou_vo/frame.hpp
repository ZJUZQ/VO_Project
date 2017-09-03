#ifndef FRAME_H
#define FRAME_H

#include "zhou_vo/common_include.hpp"
#include "zhou_vo/camera.hpp"

namespace zhou_vo{

	class Frame{
	public:
		typedef std::shared_ptr<Frame> Ptr;
	 	long 			id_; 				// id of this frame
		double 			time_stamp_;		// when this frame is recorded
		Sophus::SE3		T_cw_;				// transform from world to this camera
		Camera::Ptr 	camera_; 			// Pinhole RGBD camera model
		cv::Mat 		color_, depth_; 	// color and depth image

		Frame();
		Frame(long id, double time_stamp = 0, Sophus::SE3 T_cw = Sophus::SE3(), 
			  Camera::Ptr camera = NULL, cv::Mat color = cv::Mat(), cv::Mat depth = cv::Mat()){}
		~Frame(){}

		static Frame::Ptr createFrame();

		// find the depth
		double findDepth(const cv::KeyPoint& kp) const;

		Eigen::Vector3d getCameraCenter() const; // get camera center

		bool isInFrame(const Eigen::Vector3d& p_w) const; // check if a point is in this frame
	};
}

#endif // FRAME_H