set(PARKING_OCCUPANCY htwk_parkingOccupancy)

add_subdirectory(./../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

add_subdirectory(./../HTWK_Behavior_Tree/HTWK_LeafNode htwk_leaf_node)
include_directories(${HTWK_LEAF_LIB_INCLUDE_DIR})

add_subdirectory(./../HTWK_world_service_h htwk_world_service_h)
include_directories(${HTWK_WORLD_SERVICE_H_INCLUDE_DIR})

adtf_add_filter(${PARKING_OCCUPANCY}
        ../HTWK_Types/ParkingLot.h
        stdafx.h
        stdafx.cpp
        ../HTWK_Utils/VectorHelper.h
        ../HTWK_Utils/ImageHelper.h
        parkingOccupancy.cpp
        parkingOccupancy.h)

adtf_install_plugin(${PARKING_OCCUPANCY} ${CMAKE_INSTALL_BINARY})

adtf_set_folder(${PARKING_OCCUPANCY} ${CMAKE_INSTALL_BINARY})

target_link_libraries(${PARKING_OCCUPANCY} htwk_utils htwk_leaf_node htwk_world_service_h)


