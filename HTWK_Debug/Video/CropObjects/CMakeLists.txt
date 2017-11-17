set(FILTER_NAME HTWK_CropObjects)
include_directories(../include
        ${OpenCV_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
adtf_add_filter(${FILTER_NAME}
        stdafx.h
        CropObjects.h
        CropObjects.cpp)
adtf_set_folder(${FILTER_NAME} template)
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
