#include "zhou_vo/camera.hpp"
#include "zhou_vo/config.hpp"

namespace zhou_vo{

	Camera::Camera(){
		fx_ = Config::get<double>("camera.fx");
		fy_ = Config::get<double>("camera.fy");
		cx_ = Config::get<double>("camera.cx");
		cy_ = Config::get<double>("camera.cy");
		depth_scale_ = Config::get<double>("camera.depth_scale");
	}

	Eigen::Vector3d Camera::world2camera(const Eigen::Vector3d& p_w, const Sophus::SE3& T_cw) const{
		return T_cw * p_w;
	}

	Eigen::Vector3d Camera::camera2world(const Eigen::Vector3d& p_c, const Sophus::SE3& T_cw) const{
		return T_cw.inverse() * p_c;
	}

	Eigen::Vector2d Camera::camera2pixel(const Eigen::Vector3d& p_c) const{
		return Eigen::Vector2d(fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
							   fy_ * p_c(1, 0) / p_c(2, 0) + cy_);
	}

	Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d& p_p, double depth) const{
		return Eigen::Vector3d( (p_p(0, 0) - cx_) * depth / fx_,
								(p_p(1, 0) - cy_) * depth / fy_,
								depth);
	}

	Eigen::Vector3d Camera::pixel2world (const Eigen::Vector2d& p_p, const Sophus::SE3& T_cw, double depth) const{
		return camera2world(pixel2camera(p_p, depth), T_cw);
	}

	Eigen::Vector2d Camera::world2pixel (const Eigen::Vector3d& p_w, const Sophus::SE3& T_cw) const{
		return camera2pixel(world2camera(p_w, T_cw));
	}
}