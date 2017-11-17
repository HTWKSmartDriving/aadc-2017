set(ROI_CUTTER htwk_roi_cutter)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
#find_package(Boost REQUIRED COMPONENTS thread)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR}) # ${Boost_INCLUDE_DIRS})
#link_directories(${Boost_LIBRARY_DIR})

add_subdirectory(./../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

adtf_add_filter(${ROI_CUTTER}
        stdafx.h
        RoiCutter.cpp
        RoiCutter.h
        )

target_link_libraries(${ROI_CUTTER} ${OpenCV_LIBS} htwk_utils)

# Specify where it should be installed to
adtf_install_plugin(${ROI_CUTTER} ${CMAKE_INSTALL_BINARY})
