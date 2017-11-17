add_subdirectory(../HTWK_CompositeNode htwk_composite_node)
include_directories(${COMPOSITE_FILTER_LIB_INCLUDE_DIR})

set(FILTER_NAME htwk_bt_compositor_sequence)

adtf_add_filter(${FILTER_NAME}
        stdafx.h
        Sequence.h
        Sequence.cpp
        )

adtf_set_folder(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

target_link_libraries(${FILTER_NAME} htwk_composite_node)
