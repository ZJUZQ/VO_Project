#ifndef CAMERA_H
#define CAMERA_H

#include "zhou_vo/common_include.hpp"

namespace zhou_vo{

	//Pinhole RGBD camera model
	class Camera{
	public:
		typedef std::shared_ptr<Camera> Ptr;
		// shared_ptr是一种智能指针（smart pointer），作用有如同指针，
		// 但会记录有多少个shared_ptrs共同指向一个对象。这便是所谓的引用计数（reference counting）

		double fx_, fy_, cx_, cy_, depth_scale_;

		Camera();
		Camera(double fx, double fy, double cx, double cy, double depth_scale = 1.0) :
			fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale){}
		~Camera(){};

		//cooridnate transform: world, camera, pixel
		Eigen::Vector3d world2camera(const Eigen::Vector3d& p_w, const Sophus::SE3& T_cw) const;
		Eigen::Vector3d camera2world(const Eigen::Vector3d& p_c, const Sophus::SE3& T_cw) const;
		Eigen::Vector2d camera2pixel(const Eigen::Vector3d& p_c) const;
		Eigen::Vector3d pixel2camera(const Eigen::Vector2d& p_p, double depth = 1.0) const;
		Eigen::Vector3d pixel2world (const Eigen::Vector2d& p_p, const Sophus::SE3& T_cw, double depth = 1.0) const;
		Eigen::Vector2d world2pixel (const Eigen::Vector3d& p_w, const Sophus::SE3& T_cw) const;
	};
}

#endif 