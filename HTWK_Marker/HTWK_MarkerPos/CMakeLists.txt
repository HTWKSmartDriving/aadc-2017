set(FILTER_NAME HTWK_MarkerPos)

find_library(TINYXML tinyxml2)
find_package(OpenMP)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_subdirectory(./../../HTWK_Utils htwk_utils)

# include directories
include_directories(${OpenCV_INCLUDE_DIR} ${HTWK_UTILS_INCLUDE_DIR})

# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
        ../../HTWK_Types/RoadSignDetectionResult.h
        ../../HTWK_Types/InitalRoadSign.h
        ../../HTWK_Types/RoadSignEnum.h
        MarkerPositioningFilter.h
        MarkerPositioningFilter.cpp
)

target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS} tinyxml2 htwk_utils)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} position)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
