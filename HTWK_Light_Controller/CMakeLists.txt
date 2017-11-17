set(LIGHT_CONTROLLER htwk_lightcontroller)

add_subdirectory(./../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

add_subdirectory(./../HTWK_Behavior_Tree/HTWK_LeafNode htwk_leaf_node)
include_directories(${HTWK_LEAF_LIB_INCLUDE_DIR})

add_subdirectory(./../HTWK_world_service_h htwk_world_service_h)
include_directories(${HTWK_WORLD_SERVICE_H_INCLUDE_DIR})

adtf_add_filter(${LIGHT_CONTROLLER}
        stdafx.h
        stdafx.cpp
        LightController.cpp
        LightController.h)

adtf_install_plugin(${LIGHT_CONTROLLER} ${CMAKE_INSTALL_BINARY})

adtf_set_folder(${LIGHT_CONTROLLER} ${CMAKE_INSTALL_BINARY})

target_link_libraries(${LIGHT_CONTROLLER} htwk_utils htwk_leaf_node htwk_world_service_h)


