find_package(Eigen3 REQUIRED)
find_package(glog)

add_executable(test_utils test_utils.cpp)
target_link_libraries(test_utils ${tracking_srcs})