#include "zhou_vo/Ceres_BA_PnP.hpp"

struct ReprojectionError {
	ReprojectionError(cv::Point3d p3d_, cv::Point2d p2d_, cv::Mat K_) 
		: _p3d(p3d_), _p2d(p2d_), _K(K_) {}

	template <typename T>
	bool operator()(const T* const pose_,	// pose.head<3>: rotation;  pose.tail<3>: translation
	      			T* residuals) const {
		// pose[0,1,2] are the angle-axis rotation.
		T p[3];

		// p = R(angle_axis) * _p3d;
		T p3d[3] = {T(_p3d.x), T(_p3d.y), T(_p3d.z)};
		ceres::AngleAxisRotatePoint(pose_, p3d, p); //#include "ceres/rotation.h"

		// pose[3,4,5] are the translation.
		p[0] += pose_[3]; p[1] += pose_[4]; p[2] += pose_[5];

		T predicted_x = _K.at<T>(0, 0) * p[0] / p[2] + _K.at<T>(0, 2);
		T predicted_y = _K.at<T>(1, 1) * p[1] / p[2] + _K.at<T>(1, 2);

		// The error is the difference between the predicted and observed position.
		residuals[0] = predicted_x - T(_p2d.x);
		residuals[1] = predicted_y - T(_p2d.y);
		return true;
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(cv::Point3d p3d, cv::Point2d p2d, cv::Mat K){
		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6>(
		         new ReprojectionError(p3d, p2d, K)));
	}

	cv::Point3d _p3d;
	cv::Point2d _p2d;
	cv::Mat _K; // camera matrix
};

void bundle_adjustment_ceres(const std::vector<cv::Point3d> pts_3d,
							 const std::vector<cv::Point2d> pts_2d,
							 const cv::Mat& K,
							 Sophus::SE3& T_cr){ // pose: angleaxis, translate
	
	//	Each residual in a BAL problem depends on a six parameter: pose.
	//  pose.head<3>: rotation;  pose.tail<3>: translation

	typedef Eigen::Matrix<double, 6, 1> Vector6d;
	Vector6d se3 = T_cr.log(); // se3.head<3> = translation;	se3.tail<3> = rotation
	double camera_pose[6] = {se3(3, 0),
							 se3(4, 0),
							 se3(5, 0),
							 se3(0, 0),
							 se3(1, 0),
							 se3(2, 0)};

	//double pts_3d_array[3 * pts_3d.size()];

	ceres::Problem problem;
	for (size_t i = 0; i < pts_3d.size(); ++i) {
		ceres::CostFunction* cost_function = ReprojectionError::Create(pts_3d[i], pts_2d[i], K);

		/*pts_3d_array[3 * i] = pts_3d[i].x;
		pts_3d_array[3 * i + 1] = pts_3d[i].y;
		pts_3d_array[3 * i + 2] = pts_3d[i].z;*/

		ceres::LossFunction* loss_function  = new ceres::HuberLoss(1.0);
		problem.AddResidualBlock(cost_function,
		                       loss_function , //squared loss 
		                       camera_pose);
	}

	ceres::Solver::Options options;
	// linear solver 的选取
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
	options.dense_linear_algebra_library_type = ceres::EIGEN;

	// 下降策略的选取
	options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

	options.gradient_tolerance = 1e-16;
  	options.function_tolerance = 1e-16;
	options.minimizer_progress_to_stdout = false;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";

	for(int i = 0; i < 3; i++){
		se3(i, 0) = camera_pose[i + 3];
		se3(i + 3, 0) = camera_pose[i];
	}
	T_cr = Sophus::SE3::exp(se3);
}
