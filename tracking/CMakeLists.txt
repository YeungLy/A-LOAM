
# install to /bin by default
install(TARGETS ${CPP_FILE} DESTINATION ../bin)

include_directories(
    ./include
    )
aux_source_directory(./src tracking_SRCS)

ADD_LIBRARY(tracking ${tracking_SRCS})
