set(FILTER_NAME HTWK_Camera_Realsense)
# add the include directory of the filter
include_directories(../include
        ${REALSENSE_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS})
link_directories(${REALSENSE_LIBRARY_DIRS})
#c++11 support for librealsense
#if we use std=c++11 we have some warnings from ADTF an we supress them here
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-literal-suffix")
adtf_add_filter(${FILTER_NAME}
        stdafx.h
        ../../HTWK_Types/CameraProperties.h
        RealsenseCamera.h
        RealsenseCamera.cpp
        )
adtf_set_folder(${FILTER_NAME} template)
target_link_libraries(${FILTER_NAME} ${REALSENSE_LIBS} ${LIBUSB1_LIBRARIES} ${OpenCV_LIBS})
# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})