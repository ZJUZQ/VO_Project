add_library (zhouslam SHARED
		camera.cpp
		config.cpp
		frame.cpp
		map.cpp
		mappoint.cpp
		visual_odometry.cpp)

target_link_libraries(zhouslam ${THIRD_PARTY_LIBS})