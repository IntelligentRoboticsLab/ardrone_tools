cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
SET(CVSRC ${CMAKE_SOURCE_DIR}/src/cv_cpp)

SET(UTILSDIR ${CMAKE_SOURCE_DIR}/src/cv_cpp/utils)
include_directories(${UTILSDIR})
FILE(GLOB UTILS ${UTILSDIR}/*.cpp ${UTILSDIR}/*.hpp)
#FILE(GLOB BLOBDETECTION ${CVSRC}/blob_detection/*.cpp ${CVSRC}/blob_detection/*.hpp)
#FILE(GLOB ARDRONECTRL ${CVSRC}/ardrone_control/*.cpp ${CVSRC}/ardrone_control/*.hpp)
#FILE(GLOB CALIBRATOR ${CVSRC}/calibrator/*.cpp ${CVSRC}/calibrator/*.hpp)

# Executables for cv algorithms
#rosbuild_add_executable(follow_line ${UTILS} ${CVSRC}/line_following/main.cpp ${CVSRC}/line_following/LineFinderCamera.hpp
#                        ${CVSRC}/line_following/LineFinderCamera.cpp ${CVSRC}/line_following/LineFinder.cpp
#                        ${CVSRC}/line_following/LineFinder.hpp)
#rosbuild_add_executable(brute_force_tracker ${CVSRC}/brute_force_tracker/main.cpp
#                        ${CVSRC}/brute_force_tracker/BruteForceCamera.hpp ${CVSRC}/brute_force_tracker/BruteForceCamera.cpp
#                        ${CVSRC}/brute_force_tracker/BruteForceTracker.cpp ${CVSRC}/brute_force_tracker/BruteForceTracker.cpp)
#rosbuild_add_executable(ardrone_control ${UTILS} ${BLOBDETECTION} ${ARDRONECTRL})
#rosbuild_add_executable(Calibrator ${UTILS} ${BLOBDETECTION} ${ARDRONECTRL} ${CALIBRATOR})
