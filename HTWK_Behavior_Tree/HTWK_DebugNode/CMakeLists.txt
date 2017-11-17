set(FILTER_NAME HTWK_Debug_Node)

add_subdirectory(../../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    DebugNode.h
    DebugNode.cpp
)

adtf_set_folder(${FILTER_NAME} template)

adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

target_link_libraries(${FILTER_NAME} htwk_utils)
