#include "zhou_vo/g2o_BA_PnP.hpp"

namespace zhou_vo{
	
	void _3d2d_BA_g2o(	const std::vector<cv::Point3d> pts_3d,
						const std::vector<cv::Point2d> pts_2d,
						Frame::Ptr curr,
						cv::Mat& inliers_,
						std::vector<MapPoint::Ptr>& match_3dpts,
						Sophus::SE3& T_cw){

		/** 这里只优化相机位姿Ksi,通过最小化重投影误差来构建优化问题;
		自定义一个g2o中的优化边，只优化一个位姿，因此是一个一元边;
		*/

		typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 2> > Block;
		Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType> ();
		Block* solver_ptr = new Block(linearSolver);
		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm(solver);

		// add vertex
		g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
		pose->setId(0);
		pose->setEstimate( g2o::SE3Quat( T_cw.rotation_matrix(), T_cw.translation() ) );
		optimizer.addVertex(pose);

		// add edge
		for(int i = 0; i < inliers_.rows; i++){
			int index = inliers_.at<int>(i, 0);
			// inliers:	Output vector that contains indices of inliers in pts_3d and pts_2d .

			// 3D -> 2D projection
			EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
			edge->setId(i);
			edge->setVertex(0, pose);
			edge->camera_ = curr->camera_.get();
			edge->point_ = Eigen::Vector3d(pts_3d[index].x, pts_3d[index].y, pts_3d[index].z);
			edge->setMeasurement( Eigen::Vector2d(pts_2d[index].x, pts_2d[index].y) );
			edge->setInformation(Eigen::Matrix2d::Identity());
			optimizer.addEdge(edge);
			//set the inlier map points
			match_3dpts[index]->matched_times_++;
		}

		optimizer.initializeOptimization();
		optimizer.optimize(50);

		T_cw = Sophus::SE3( pose->estimate().rotation(), pose->estimate().translation() );
	}

}