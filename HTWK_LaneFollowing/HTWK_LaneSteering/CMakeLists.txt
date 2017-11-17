# add the include directory of the filter
#include_directories(../include)
# declare our filter
set(FILTER_NAME HTWK_LaneSteering)

add_subdirectory(./../../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

adtf_add_filter(${FILTER_NAME}
        stdafx.h
        ../../HTWK_Types/LaneData.h
        ../../HTWK_Types/CameraResolutionData.h
        ../../HTWK_Utils/HTWKPoint.hpp
        ../../HTWK_Utils/HTWKPoint.cpp
        LaneSteeringCalculator.h
        LaneSteeringCalculator.cpp
        LaneSteeringFilter.h
        LaneSteeringFilter.cpp
        )

adtf_set_folder(${FILTER_NAME} template)

target_link_libraries(${FILTER_NAME} htwk_utils)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
