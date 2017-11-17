set(HTWK_CONTROLLER_PID htwk_controllerPID)

adtf_add_filter(${HTWK_CONTROLLER_PID}
        stdafx.h
        stdafx.cpp
        HTWKControllerPID.h
        HTWKControllerPID.cpp
        )

adtf_install_plugin(${HTWK_CONTROLLER_PID} ${CMAKE_INSTALL_BINARY})

adtf_set_folder(${HTWK_CONTROLLER_PID} ${CMAKE_INSTALL_BINARY})