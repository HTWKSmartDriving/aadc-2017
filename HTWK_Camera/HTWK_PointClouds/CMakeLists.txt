cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
project(HTWK_PointClouds)
set(FILTER_NAME ${PROJECT_NAME})

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${REALSENSE_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)

include_directories(${HTWK_UTILS_INCLUDE_DIR})

link_directories(
        ${REALSENSE_LIBRARY_DIRS}
)

set(HEADER
        stdafx.h
        PointClouds.h
)

set(SOURCE
        PointClouds.cpp
)

include_directories(
        ${Boost_INCLUDE_DIR}
)

adtf_add_filter(${FILTER_NAME} ${HEADER} ${SOURCE})

target_link_libraries(${FILTER_NAME}
        ${REALSENSE_LIBS}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        htwk_utils
)

find_package(PCL REQUIRED 1.7.2
        COMPONENTS filters segmentation visualization
        PATHS "/opt/pcl/1.8.1/share/pcl-1.8")
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(${FILTER_NAME} ${PCL_LIBRARIES})

adtf_set_folder(${FILTER_NAME} template)

adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
