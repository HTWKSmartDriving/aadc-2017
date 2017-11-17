set(FILTER_NAME HTWK_markerDetector)

add_subdirectory(./../../HTWK_Utils htwk_utils)
include_directories(${OpenCV_INCLUDE_DIR} ${HTWK_UTILS_INCLUDE_DIR})

# add the source
adtf_add_filter(${FILTER_NAME}
        aruco_helpers.h
        ../../HTWK_Utils/ImageHelper.h
        ../../HTWK_Types/RoadSignDetectionResult.h
        MarkerDetectorFilter.h
        MarkerDetectorFilter.cpp
        )

# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS} htwk_utils)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} marker)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
