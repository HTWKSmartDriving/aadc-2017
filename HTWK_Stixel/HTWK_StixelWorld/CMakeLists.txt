set(FILTER_NAME HTWK_StixelWorld)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})

find_package(OpenMP)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_subdirectory(./../../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

adtf_add_filter(${FILTER_NAME}
        stdafx.h
        ../../HTWK_Types/CameraProperties.h
        ../../HTWK_Types/StixelData.h
        worldOfStixel/imageBasedFreeSpaceComputation/imageBasedFreeSpaceComputation.h
        worldOfStixel/imageBasedFreeSpaceComputation/imageBasedFreeSpaceComputation.cpp
        worldOfStixel/heightSegmentation/heightSegmentation.h
        worldOfStixel/heightSegmentation/heightSegmentation.cpp
        worldOfStixel/worldOfStixel.h
        worldOfStixel/worldOfStixel.cpp
        StixelWorldFilter.h
        StixelWorldFilter.cpp
        )

target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS} htwk_utils)

adtf_set_folder(${FILTER_NAME} template)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

#install dlls for opencv to binary folder 
if (WIN32)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Debug)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS RelWithDebInfo Release)

    install(FILES "${Qt_DIR}/bin/QtTest4.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Release RelWithDebInfo)
    install(FILES "${Qt_DIR}/bin/QtTestd4.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS Debug)
endif (WIN32)
