set(MAP_WORKER htwk_mapworker)

find_library(TINYXML tinyxml2)

add_subdirectory(./../HTWK_Map htwk_map)
include_directories(${HTWK_MAP_INCLUDE_DIR})

add_subdirectory(./../HTWK_Car_Trajectory htwk_car_trajectory)
include_directories(${CAR_TRAJECTORY_INCLUDE_DIR})

add_subdirectory(./../HTWK_Utils htwk_utils)
include_directories(${HTWK_UTILS_INCLUDE_DIR})

add_subdirectory(./../HTWK_MapPredictor htwk_map_predictor)
include_directories(${HTWK_MAP_PREDICTOR})

adtf_add_filter(${MAP_WORKER}
        stdafx.h
        stdafx.cpp
        MapWorker.cpp
        MapWorker.h)

adtf_install_plugin(${MAP_WORKER} ${CMAKE_INSTALL_BINARY})

adtf_set_folder(${MAP_WORKER} ${CMAKE_INSTALL_BINARY})

target_link_libraries(${MAP_WORKER} ${OpenCV_LIBS} htwk_map htwk_car_trajectory tinyxml2 htwk_utils htwk_map_predictor)


