add_subdirectory(./../HTWK_world_service_h htwk_world_service_h)
include_directories(${HTWK_WORLD_SERVICE_H_INCLUDE_DIR})

add_subdirectory(../HTWK_Behavior_Tree/HTWK_LeafNode htwk_leaf_node)
include_directories(${COMPOSITE_FILTER_LIB_INCLUDE_DIR})

set(FILTER_NAME htwk_brake)

adtf_add_filter(${FILTER_NAME}
        stdafx.h
        Brake.h
        Brake.cpp
        )

adtf_set_folder(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

target_link_libraries(${FILTER_NAME} htwk_leaf_node)
