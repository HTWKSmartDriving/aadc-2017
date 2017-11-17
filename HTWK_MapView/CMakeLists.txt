set(MAP_VIEW htwk_mapview)

find_library(TINYXML tinyxml2)

add_subdirectory(./../HTWK_Map htwk_map)
include_directories(${HTWK_MAP_INCLUDE_DIR})

add_subdirectory(./../HTWK_Car_Trajectory htwk_car_trajectory)
include_directories(${CAR_TRAJECTORY_INCLUDE_DIR})

add_subdirectory(./../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

add_subdirectory(./../HTWK_Behavior_Tree/HTWK_LeafNode htwk_leaf_node)
include_directories(${HTWK_LEAF_LIB_INCLUDE_DIR})

add_subdirectory(./../HTWK_world_service_h htwk_world_service_h)
include_directories(${HTWK_WORLD_SERVICE_H_INCLUDE_DIR})

adtf_add_filter(${MAP_VIEW}
        stdafx.h
        stdafx.cpp
        MapView.cpp
        MapView.h)

adtf_install_plugin(${MAP_VIEW} ${CMAKE_INSTALL_BINARY})

adtf_set_folder(${MAP_VIEW} ${CMAKE_INSTALL_BINARY})

target_link_libraries(${MAP_VIEW} ${OpenCV_LIBS} htwk_map htwk_car_trajectory tinyxml2 htwk_utils htwk_leaf_node htwk_world_service_h)


