add_subdirectory(../../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

set(DECORATOR_FILTER_LIB htwk_decorator_node)
set(DECORATOR_FILTER_LIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

if (TARGET ${DECORATOR_FILTER_LIB})
    message("${DECORATOR_FILTER_LIB} is already defined")
else ()
    ADD_LIBRARY(${DECORATOR_FILTER_LIB} STATIC
            stdafx.h
            DecoratorNode.h
            DecoratorNode.cpp
            ../../HTWK_Types/BehaviorTreeData.h)

    target_link_libraries(${DECORATOR_FILTER_LIB} ${ADTF_LIBS} htwk_utils)
endif ()