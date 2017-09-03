#include "zhou_vo/config.hpp"

namespace zhou_vo{
	void Config::setParameterFile(const std::string& filename){
		if(config_ == NULL)
			config_ = std::shared_ptr<Config>(new Config);
		config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
		if(config_->file_.isOpened() == false){
			std::cout << "parameter file " << filename << " does not exist." << std::endl;
			config_->file_.release();
			return;
		}
	}

	Config::~Config(){
		if(file_.isOpened())
			file_.release();
	}

	std::shared_ptr<Config> Config::config_ = NULL; // 单件模式的全局指针
}