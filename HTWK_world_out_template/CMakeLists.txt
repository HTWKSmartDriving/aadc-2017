set(WORLD_OUT_TEMPLATE htwk_world_out_template_filter)

find_library(TINYXML tinyxml2)

add_subdirectory(./../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

add_subdirectory(./../HTWK_world_service_h htwk_world_service_h)
include_directories(${HTWK_WORLD_SERVICE_H_INCLUDE_DIR})

add_subdirectory(./../HTWK_world_out_filter_base htwk_htwk_filter_out_lib)
include_directories(${HTWK_FILTER_OUT_LIB_INCLUDE_DIR})

add_subdirectory(./../HTWK_Map htwk_map)
include_directories(${HTWK_MAP_INCLUDE_DIR})

adtf_add_filter(${WORLD_OUT_TEMPLATE}
        stdafx.h
        WorldOutTemplateFilter.cpp
        WorldOutTemplateFilter.h)

adtf_install_plugin(${WORLD_OUT_TEMPLATE} ${CMAKE_INSTALL_BINARY})

adtf_set_folder(${WORLD_OUT_TEMPLATE} ${CMAKE_INSTALL_BINARY})

target_link_libraries(${WORLD_OUT_TEMPLATE} htwk_map htwk_world_service_h htwk_utils htwk_filter_out_lib tinyxml2)
