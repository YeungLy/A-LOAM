
# install to /bin by default
install(TARGETS ${CPP_FILE} DESTINATION ../bin)

include_directories(
    .
    )
aux_source_directory(. kitti_SRCS)

ADD_LIBRARY(kitti ${kitti_SRCS})
