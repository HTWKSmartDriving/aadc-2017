#
# Misc
#

set(FILTER_OUT htwk_world_filter_out)

add_subdirectory(./../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

add_subdirectory(./../HTWK_world_service_h htwk_world_service_h)
include_directories(${HTWK_WORLD_SERVICE_H_INCLUDE_DIR})

find_library(TINYXML tinyxml2)

#
# Filter In
#

adtf_add_filter(${FILTER_OUT}
        stdafx.h
        WorldFilterOut.h
        WorldFilterOut.cpp)

adtf_install_plugin(${FILTER_OUT} ${CMAKE_INSTALL_BINARY})

adtf_set_folder(${FILTER_OUT} ${CMAKE_INSTALL_BINARY})

target_link_libraries(${FILTER_OUT} htwk_world_service_h htwk_utils tinyxml2)
