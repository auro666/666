cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set (DC  libdc1394-2)
set (TRICLOPS triclops3.2)

find_package(PkgConfig)

pkg_check_modules(DC libdc1394-2)
link_directories(${DC_LIBRARY_DIRS})
include_directories(${DC_INCLUDE_DIRS})

pkg_check_modules(TRICLOPS triclops3.2)
link_directories(${TRICLOPS_LIBRARY_DIRS})
include_directories(${TRICLOPS_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

LINK_DIRECTORIES("/usr/local/lib/")

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

rosbuild_add_executable(stereopsis src/stereopsis.cpp)
target_link_libraries(stereopsis  ${DC_LIBRARIES} ${TRICLOPS_LIBRARIES})

rosbuild_add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher pthread rt ${OpenCV_LIBS})

rosbuild_add_executable(tester src/tester.cpp)
target_link_libraries(tester pthread rt ${OpenCV_LIBS})

