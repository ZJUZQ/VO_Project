//--------------------------- test the visual odometry -------------------------//
#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>

#include "zhou_vo/config.hpp"
#include "zhou_vo/visual_odometry.hpp"

using namespace std;

int main(int argc, char** argv){
	if(argc != 2){
		cout << "usage: bin/run_vo config/default.yaml" << endl;
		return 1;
	}
	zhou_vo::Config::setParameterFile(argv[1]);
	zhou_vo::VisualOdometry::Ptr vo(new zhou_vo::VisualOdometry);

	string dataset_dir = zhou_vo::Config::get<string>("dataset_dir");
	cout << "dataset: " << dataset_dir << endl;
	ifstream fin((dataset_dir + "/associate.txt").c_str());
	if(!fin){
		cout << "please generate the associate file called associate.txt!" << endl;
		return 1;
	}
	vector<string> rgb_files, depth_files;
	vector<double> rgb_times, depth_times;
	while(!fin.eof()){
		string rgb_time, rgb_file, depth_time, depth_file;
		fin >> rgb_time >> rgb_file >> depth_time >> depth_file;
		rgb_times.push_back(atof(rgb_time.c_str()));
		depth_times.push_back(atof(depth_time.c_str()));
		rgb_files.push_back(dataset_dir + "/" + rgb_file);
		depth_files.push_back(dataset_dir + "/" + depth_file);
		if(fin.good() == false)
			break;
	}

	zhou_vo::Camera::Ptr camera(new zhou_vo::Camera);

	// visualization
	cv::viz::Viz3d vis("visual odometry");
	cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
	cv::Point3d cam_pos(0, -1.0, -1.0), cam_focal_point(0, 0, 0), cam_y_dir(0, 1, 0);
	cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
	vis.setViewerPose(cam_pose);

	world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
	camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
	vis.showWidget("world", world_coor);
	vis.showWidget("camera", camera_coor);

	cout << "read total " << rgb_files.size() << " entries" << endl;
	cv::namedWindow("image");

	for(size_t i = 0; i < rgb_files.size(); i++){
		clock_t t1 = clock();

		cv::Mat color = cv::imread(rgb_files[i]);
		cv::Mat depth = cv::imread(depth_files[i], -1);
		if(color.data == NULL || depth.data == NULL)
			continue;

		zhou_vo::Frame::Ptr pframe = zhou_vo::Frame::createFrame();
		pframe->camera_ = camera;
		pframe->color_ = color;
		pframe->depth_ = depth;
		pframe->time_stamp_ = rgb_times[i];

		vo->addFrame(pframe);
		if(vo->state_ == zhou_vo::VisualOdometry::LOST)
			break;
		Sophus::SE3 Twc = pframe->T_cw_.inverse();

		// show the map and the camera pose
		// Affine3 (const Mat3 &R, const Vec3 &t=Vec3::all(0))
		cv::Affine3d M(
            cv::Affine3d::Mat3( 
                Twc.rotation_matrix()(0,0), Twc.rotation_matrix()(0,1), Twc.rotation_matrix()(0,2),
                Twc.rotation_matrix()(1,0), Twc.rotation_matrix()(1,1), Twc.rotation_matrix()(1,2),
                Twc.rotation_matrix()(2,0), Twc.rotation_matrix()(2,1), Twc.rotation_matrix()(2,2) ), 
            cv::Affine3d::Vec3(
                Twc.translation()(0,0), Twc.translation()(1,0), Twc.translation()(2,0)) );

		cout << "VO cost time: " << (clock() - t1) * 1000 / CLOCKS_PER_SEC << endl;

		cv::imshow("image", color);
		cv::waitKey(1);
		vis.setWidgetPose("camera", M);
		vis.spinOnce(1, false);
	}
	cv::destroyAllWindows();
	return 0;
}