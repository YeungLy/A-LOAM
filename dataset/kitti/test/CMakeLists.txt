set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Eigen3 REQUIRED)
find_package(glog)

include_directories(
    "/usr/include/eigen3"
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/.. kitti_srcs)
add_executable(test_iou test_iou.cpp ${kitti_srcs})
target_link_libraries(test_iou glog)
