set(COMPOSITE_FILTER_LIB htwk_composite_node)
set(COMPOSITE_FILTER_LIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

if (TARGET ${COMPOSITE_FILTER_LIB})
    message("${COMPOSITE_FILTER_LIB} is already defined")
else ()
    ADD_LIBRARY(${COMPOSITE_FILTER_LIB} STATIC
            stdafx.h
            CompositeNode.h
            CompositeNode.cpp
            ../../HTWK_Types/BehaviorTreeData.h)

    target_link_libraries(${COMPOSITE_FILTER_LIB} ${ADTF_LIBS})
endif ()