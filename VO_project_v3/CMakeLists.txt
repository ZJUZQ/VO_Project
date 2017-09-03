cmake_minimum_required (VERSION 3.0)
project (myslam)

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_COMPILER "g++")
set (CMAKE_BUILD_TYPE "Release")

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set (LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

####################### dependencies ##########################
#Eigen
include_directories ("/usr/include/eigen3")

#OpenCV
find_package (OpenCV REQUIRED)
include_directories (${OpenCV_INCLUDE_DIRS})

#Sophus
find_package(Sophus REQUIRED)
include_directories (${Sophus_INCLUDE_DIRS})

#Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

set (THIRD_PARTY_LIBS
	${OpenCV_LIBS}
	${Sophus_LIBRARIES}
	${CERES_LIBRARIES})

######################### source and test #######################
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory (src)
add_subdirectory (test)
