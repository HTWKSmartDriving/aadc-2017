set(MAP htwk_map)

set(HTWK_MAP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

include_directories(./../lib/tinyxml2 tinyxml)

if (TARGET ${MAP})
    message("${MAP} is already defined")
else ()
    file(GLOB_RECURSE FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
    file(GLOB_RECURSE HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

    ADD_LIBRARY(${MAP} STATIC ${FILES} ${HEADERS})
endif ()