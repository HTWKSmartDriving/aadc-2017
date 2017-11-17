#
# Misc
#

set(FILTER_IN htwk_world_filter_in)

find_library(TINYXML tinyxml2)

add_subdirectory(./../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

add_subdirectory(./../HTWK_world_service_h htwk_world_service_h)
include_directories(${HTWK_WORLD_SERVICE_H_INCLUDE_DIR})

add_subdirectory(./../HTWK_Map htwk_map)
include_directories(${HTWK_MAP_INCLUDE_DIR})

#
# Filter In
#

adtf_add_filter(${FILTER_IN}
        stdafx.h
        ../HTWK_Types/ManueverEnum.h
        ../HTWK_Types/ParkingLot.h
        ../HTWK_Types/ObstacleData.h
        ../HTWK_Types/PoliceData.h
        ../HTWK_Types/IntersectionState.h
        ../HTWK_Types/RoadSignDetectionResult.h
        ../HTWK_Utils/VectorHelper.h
        ../HTWK_Utils/ImageHelper.h
        WorldFilterIn.h
        WorldFilterIn.cpp)

adtf_install_plugin(${FILTER_IN} ${CMAKE_INSTALL_BINARY})

adtf_set_folder(${FILTER_IN} ${CMAKE_INSTALL_BINARY})

target_link_libraries(${FILTER_IN} htwk_map htwk_world_service_h htwk_utils tinyxml2)
