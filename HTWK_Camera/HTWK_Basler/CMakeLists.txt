set(FILTER_NAME HTWK_Camera_Basler)
# add the include directory of the filter
include_directories(${PYLON_INCLUDE_DIRS})
link_directories(${PYLON_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-literal-suffix")
adtf_add_filter(${FILTER_NAME}
        BaslerCamera.h
        BaslerCamera.cpp
        )
adtf_set_folder(${FILTER_NAME} template)
target_link_libraries(${FILTER_NAME} ${PYLON_LIBS})
# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})

