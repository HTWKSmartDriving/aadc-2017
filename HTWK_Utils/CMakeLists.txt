set(UTILS htwk_utils)

set(HTWK_UTILS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

if (TARGET ${UTILS})
    message("${UTILS} is already defined")
else ()
    file(GLOB_RECURSE FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
    file(GLOB_RECURSE HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

    ADD_LIBRARY(${UTILS} STATIC ${FILES} ${HEADERS})

    include_directories(${OpenCV_INCLUDE_DIR})

    target_link_libraries(${UTILS} ${OpenCV_LIBS})
endif ()
