set(MAP_DRIVER htwk_mapdriver)

find_library(TINYXML tinyxml2)

add_subdirectory(./../HTWK_Map htwk_map)
include_directories(${HTWK_MAP_INCLUDE_DIR})

add_subdirectory(./../HTWK_Car_Trajectory htwk_car_trajectory)
include_directories(${CAR_TRAJECTORY_INCLUDE_DIR})

add_subdirectory(./../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

adtf_add_filter(${MAP_DRIVER}
        stdafx.h
        stdafx.cpp
        MapDriver.cpp
        MapDriver.h)

adtf_install_plugin(${MAP_DRIVER} ${CMAKE_INSTALL_BINARY})

adtf_set_folder(${MAP_DRIVER} ${CMAKE_INSTALL_BINARY})

target_link_libraries(${MAP_DRIVER} ${OpenCV_LIBS} htwk_map htwk_car_trajectory tinyxml2 htwk_utils)


