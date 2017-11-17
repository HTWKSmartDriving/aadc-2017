set(HTWK_MAP_PREDICTOR htwk_map_predictor)

set(HTWK_MAP_PREDICTOR_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

if (TARGET ${HTWK_MAP_PREDICTOR})
    message("${HTWK_MAP_PREDICTOR} is already defined")
else ()
    file(GLOB_RECURSE FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
    file(GLOB_RECURSE HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

    ADD_LIBRARY(${HTWK_MAP_PREDICTOR} STATIC ${FILES} ${HEADERS})

    find_library(TINYXML tinyxml2)

    target_link_libraries(${HTWK_MAP_PREDICTOR} tinyxml2)
endif ()
