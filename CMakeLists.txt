cmake_minimum_required(VERSION 2.8.3)
project(obj_aloam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
  jsk_recognition_msgs
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(glog)

find_package(Boost REQUIRED COMPONENTS
        serialization)

include_directories(
  include
    ${PROJECT_SOURCE_DIR}/tracking/include
    ${PROJECT_SOURCE_DIR}/dataset
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})

add_subdirectory(tracking)
add_subdirectory(dataset/kitti)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs jsk_recognition_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)


add_executable(aSegment src/segmentCloud.cpp)
target_link_libraries(aSegment ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(aTracklet src/object.cpp)
target_link_libraries(aTracklet ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} tracking glog kitti)

add_executable(aTestTracklet src/test_tracking.cpp)
target_link_libraries(aTestTracklet ${catkin_LIBRARIES} ${PCL_LIBRARIES} tracking glog kitti)

add_executable(ascanRegistration src/scanRegistration.cpp)
target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(alaserOdometry src/laserOdometry.cpp)
target_link_libraries(alaserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserMapping src/laserMapping.cpp)
target_link_libraries(alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

#add_executable(kittiHelper src/kittiHelper.cpp)
#target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(kittiHelperRaw src/kittiHelperRaw.cpp)
target_link_libraries(kittiHelperRaw ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} kitti)

add_executable(kittiConverter src/kittiConverter.cpp)
target_link_libraries(kittiConverter ${catkin_LIBRARIES} kitti)
#add_executable(pubTracklet src/pubTracklet.cpp)
#target_link_libraries(pubTracklet ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(subTracklet src/subTracklet.cpp)
#target_link_libraries(subTracklet ${catkin_LIBRARIES} ${PCL_LIBRARIES})



