cmake_minimum_required(VERSION 3.0.2)
project(radar_on_lidar_map)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
        cv_bridge
)

set(OpenCV_DIR "/home/myk/dev/app/m_lib/opencv-4.5.1/build")
set(GTSAM_DIR "/home/myk/dev/app/m_lib/gtsam/build")
set(Sophus_DIR "/home/myk/dev/app/m_lib/Sophus/build")
find_package(GTSAM REQUIRED QUIET)
find_package(OpenCV 4 REQUIRED)
find_package(PCL REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
        ${catkin_INCLUDE_DIRS}
        "/usr/include/eigen3"
        "/usr/local/include"
        "/usr/include"
        "/home/myk/.local/include"
        include
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIR}
)

catkin_package(
        CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
        DEPENDS EIGEN3  # PCL
        INCLUDE_DIRS include
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(scancontext src/scancontext.cpp)
#add_library(posegraph src/poseGraph.cpp)

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
link_libraries("/usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.3.so.6.3")


add_executable(radar_lidar_scancontext src/radarLidarScanContext.cpp)
add_dependencies(radar_lidar_scancontext ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_lidar_scancontext
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${VTK_LIBRARIES_DIR}
        ${OpenCV_LIBRARIES}
        gtsam
#        posegraph
        scancontext
        )



add_executable(pclfiltertest src/pclfiltertest.cpp)
add_dependencies(pclfiltertest ${catkin_EXPORTED_TARGETS})
target_link_libraries(pclfiltertest
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        #        ${PCL_IO_LIBRARIES}
        ${VTK_LIBRARIES_DIR}
        ${OpenCV_LIBRARIES}
        gtsam
        #        posegraph
        scancontext
        )

#add_executable(generlizeMap src/generlizeMap.cpp)
#target_link_libraries(generlizeMap
#        ${PCL_LIBRARIES}
#        ${OpenCV_LIBRARIES}
#        scancontext
#        )



