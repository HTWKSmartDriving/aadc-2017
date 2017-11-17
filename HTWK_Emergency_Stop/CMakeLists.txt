include_directories(../include)
set(FILTER_NAME htwk_emergency_filter)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    EmergenyStop.h
    EmergenyStop.cpp
)

adtf_set_folder(${FILTER_NAME} template) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})