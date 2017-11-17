set(FILTER_NAME HTWK_PRE_OBSTACLE_FILTER)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})

find_package(OpenMP)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_subdirectory(./../../../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

adtf_add_filter(${FILTER_NAME}
        ../../../HTWK_Utils/VectorHelper.h
        ../../../HTWK_Types/ObstacleData.h
        ../../../HTWK_Types/InitalRoadSign.h
        PreObstacleFilter.h
        PreObstacleFilter.cpp
        )

target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS} htwk_utils)

adtf_set_folder(${FILTER_NAME} template)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

