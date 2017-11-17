add_subdirectory(./../HTWK_Behavior_Tree/HTWK_LeafNode htwk_leaf_node)
include_directories(${HTWK_LEAF_LIB_INCLUDE_DIR})

add_subdirectory(./../HTWK_world_service_h htwk_world_service_h)
include_directories(${HTWK_WORLD_SERVICE_H_INCLUDE_DIR})

#==================================================================

set(STEERING_FUSION htwk_steering_fusion)

adtf_add_filter(${STEERING_FUSION}
        stdafx.h
        stdafx.cpp
        SteeringFusion.h
        SteeringFusion.cpp)

adtf_install_plugin(${STEERING_FUSION} ${CMAKE_INSTALL_BINARY})
adtf_set_folder(${STEERING_FUSION} ${CMAKE_INSTALL_BINARY})

target_link_libraries(${STEERING_FUSION} htwk_leaf_node htwk_world_service_h)

#==================================================================

set(WORLD_LANE_OUT htwk_world_out)

adtf_add_filter(${WORLD_LANE_OUT}
        stdafx.h
        stdafx.cpp
        LaneWorldOut.cpp
        LaneWorldOut.h)

adtf_install_plugin(${WORLD_LANE_OUT} ${CMAKE_INSTALL_BINARY})
adtf_set_folder(${WORLD_LANE_OUT} ${CMAKE_INSTALL_BINARY})

target_link_libraries(${WORLD_LANE_OUT} htwk_leaf_node htwk_world_service_h)


