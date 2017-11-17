#include_directories(../include)
set(FILTER_NAME HTWK_Decorator_Inverter)

add_subdirectory(../../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

adtf_add_filter(${FILTER_NAME}
    ../HTWK_DecoratorNode/DecoratorNode.h
    ../HTWK_DecoratorNode/DecoratorNode.cpp
    stdafx.h
    Inverter.h
    Inverter.cpp
)

adtf_set_folder(${FILTER_NAME} template)

adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

target_link_libraries(${FILTER_NAME} htwk_utils)
