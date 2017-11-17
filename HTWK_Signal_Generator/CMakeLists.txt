set(SIGNAL_GEN HTWK_Signal_Generator)

adtf_add_filter(${SIGNAL_GEN}
        stdafx.h
        stdafx.cpp
        SignalGenerator.h
        SignalGenerator.cpp)

adtf_install_plugin(${SIGNAL_GEN} ${CMAKE_INSTALL_BINARY})

adtf_set_folder(${SIGNAL_GEN} ${CMAKE_INSTALL_BINARY})
