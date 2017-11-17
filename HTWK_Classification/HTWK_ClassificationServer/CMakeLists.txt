cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)
# set project name at beginning this is cleaner than just a folder
project(HTWK_ClassificationServer)
set(FILTER_NAME ${PROJECT_NAME})

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${Boost_INCLUDE_DIR} 
    ${Thrift_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ThriftInterface/cpp
)

set(HEADER
        stdafx.h
        ClassificationServer.h
        ThriftInterface/cpp/ThriftInterface_constants.h
        ThriftInterface/cpp/ThriftInterface_types.h
        ThriftInterface/cpp/ThriftInterface_Service.h)

set(SOURCE
        ClassificationServer.cpp
        ThriftInterface/cpp/ThriftInterface_constants.cpp
        ThriftInterface/cpp/ThriftInterface_types.cpp
        ThriftInterface/cpp/ThriftInterface_Service.cpp)

include_directories(
        ${Thrift_INCLUDE_DIR}
        ${Boost_INCLUDE_DIR})

adtf_add_filter(${FILTER_NAME} ${HEADER} ${SOURCE})

target_link_libraries(${FILTER_NAME}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${Thrift_LINK_LIBS}
    ${OpenSSL_LINK_LIBS}
)

adtf_set_folder(${FILTER_NAME} template)

adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
