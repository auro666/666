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
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

# initilaize boost directory search
rosbuild_add_boost_directories()

include_directories ("${PROJECT_SOURCE_DIR}/include/")

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})
link_directories("/usr/local/lib/")

rosbuild_add_executable(road_navigation src/road_navigation/road_navigation.cpp src/road_navigation/clothoid.cpp src/local_planner.cpp)
rosbuild_link_boost(road_navigation thread system signals)

rosbuild_add_executable(road_navigation_tester src/road_navigation/road_navigation_tester.cpp)

rosbuild_add_executable(freeform_navigation src/freeform_navigation/freeform_navigation.cpp)
target_link_libraries(freeform_navigation ${SBPL_LIBRARIES})

rosbuild_add_executable(freeform_navigation_tester src/freeform_navigation/freeform_navigation_tester.cpp)
