add_subdirectory(../../HTWK_world_service_h htwk_world_service_h)
include_directories(${HTWK_WORLD_SERVICE_H_INCLUDE_DIR})

find_library(TINYXML tinyxml2)

set(LEAF_FILTER_LIB htwk_leaf_node)

if (TARGET ${LEAF_FILTER_LIB})
    message("${LEAF_FILTER_LIB} is already defined")
else ()
    ADD_LIBRARY(${LEAF_FILTER_LIB} STATIC
            stdafx.h
            Leaf.h
            Leaf.cpp
            ../../HTWK_Types/BehaviorTreeData.h)

    target_link_libraries(${LEAF_FILTER_LIB} ${ADTF_LIBS} htwk_world_service_h tinyxml2)
endif ()
