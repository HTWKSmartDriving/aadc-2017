set(WORLD_H_LIB htwk_world_service_h)

set(HTWK_WORLD_SERVICE_H_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(HTWK_WORLD_SERVICE_H_INCLUDE_DIR "${HTWK_WORLD_SERVICE_H_INCLUDE_DIR};${OpenCV_INCLUDE_DIR}" PARENT_SCOPE)

include_directories(${OpenCV_INCLUDE_DIR})

if (TARGET ${WORLD_H_LIB})
    message("${WORLD_H_LIB} is already defined")
else ()
    ADD_LIBRARY(${WORLD_H_LIB} STATIC
            stdafx.h
            stdafx.cpp
            tWorldData.h
            WorldService.h)
endif ()