set(SIGNAL_GEN HTWK_State_Signal_Generator)

add_subdirectory(./../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

adtf_add_filter(${SIGNAL_GEN}
        stdafx.h
        StateSignalGenerator.h
        StateSignalGenerator.cpp)

adtf_install_plugin(${SIGNAL_GEN} ${CMAKE_INSTALL_BINARY})

adtf_set_folder(${SIGNAL_GEN} ${CMAKE_INSTALL_BINARY})

target_link_libraries(${SIGNAL_GEN} htwk_utils)
