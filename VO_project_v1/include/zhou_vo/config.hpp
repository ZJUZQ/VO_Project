#ifndef CONFIG_H
#define CONFIG_H

#include "zhou_vo/common_include.hpp"

namespace zhou_vo{
	class Config{
	private:
		static std::shared_ptr<Config> config_;
		cv::FileStorage file_;

		Config(){} // // private constructor makes a singleton
	public:
		~Config();

		static void setParameterFile(const std::string& filename);	// set a new config file

		// access the parameter values
		template<typename T>
		static T get(const std::string& key){
			return T(Config::config_->file_[key]);
		}
	};
}

#endif // CONFIG_H