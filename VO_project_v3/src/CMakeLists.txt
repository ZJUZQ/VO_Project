add_library (zhouslam SHARED
		camera.cpp
		config.cpp
		frame.cpp
		map.cpp
		mappoint.cpp
		visual_odometry.cpp
		Ceres_BA_PnP.cpp
		g2o_types.cpp
		g2o_BA_PnP.cpp)

target_link_libraries(zhouslam ${THIRD_PARTY_LIBS})