#
# Misc
#

include_directories(${OpenCV_INCLUDE_DIR})

add_subdirectory(./../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

add_subdirectory(./../HTWK_world_service_h htwk_world_service_h)
include_directories(${HTWK_WORLD_SERVICE_H_INCLUDE_DIR})

find_library(TINYXML tinyxml2)


#
# Filter Out Lib
#

set(FILTER_OUT_LIB htwk_filter_out_lib)

set(HTWK_FILTER_OUT_LIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

if (TARGET ${FILTER_OUT_LIB})
    message("${FILTER_OUT_LIB} is already defined")
else ()
    ADD_LIBRARY(${FILTER_OUT_LIB} STATIC
            stdafx.h
            WorldOutBaseFilter.h
            WorldOutBaseFilter.cpp)

    target_link_libraries(${FILTER_OUT_LIB} htwk_world_service_h tinyxml2)
    target_link_libraries(${FILTER_OUT_LIB} ${ADTF_LIBS})
endif ()
