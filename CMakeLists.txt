cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-literal-suffix -Wno-unused-function -D_GLIBCXX_USE_CXX11_ABI=0 -O3 -march=native")

cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0011 NEW)

# The project and cmake vars must only be defined in case that only the base
# filter should be build.
if(NOT AADC_BUILD_ALL_IN_ONE)
    # define a project name
    project(aadc_user)

    # CMAKE_BUILD_TYPE is empty on Unix systems, but needed by ADTF. Otherwise the ADTF libs
    # can't be found and linked.
    if(UNIX AND NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
                "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
                FORCE)
    endif(UNIX AND NOT CMAKE_BUILD_TYPE)

endif(NOT AADC_BUILD_ALL_IN_ONE)

set(ADTF_FOUND FALSE)
set(ADTF_DIR /opt/adtf/2.14.0/)
find_package(ADTF REQUIRED)
#-------General------------

set(AADC_FOUND FALSE)
set(AADC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../)
find_package(AADC REQUIRED)
if (AADC_FOUND)
   message("-- AADC found")
else(AADC_FOUND)
   message(FATAL_ERROR "-- AADC not found")
endif (AADC_FOUND)

#--------------add private macros-----------------
include(${AADC_DIR}/AADC_PRIVATE.cmake)
include(${AADC_DIR}/AADCConfigVersion.cmake)

#--------------sources----------------------------
include_directories(${AADC_DIR}/include)

#Lib
add_subdirectory(lib)
add_subdirectory(HTWK_Utils)

#Templates
#add_subdirectory(AADC_TemplateFilter)
#add_subdirectory(AADC_OpenCVTemplate)

#Realsense Camera
add_subdirectory(HTWK_Camera)
#StixelWorld
add_subdirectory(HTWK_Stixel)
#Lane Following
add_subdirectory(HTWK_LaneFollowing)
add_subdirectory(HTWK_Debug)
add_subdirectory(HTWK_LaneSteering)

#Map
add_subdirectory(HTWK_Map)
add_subdirectory(HTWK_MapView)
#add_subdirectory(HTWK_MapDriver) #working but deprecated
add_subdirectory(HTWK_MapWorker)
add_subdirectory(HTWK_Marker)
add_subdirectory(HTWK_MapPredictor)

#Trajectory
add_subdirectory(HTWK_Car_Trajectory)

#Parking
add_subdirectory(HTWK_Parking)

#Parking Occupancy
add_subdirectory(HTWK_Parking_Occupancy)

#World Service
add_subdirectory(HTWK_world_service_h)
add_subdirectory(HTWK_world_service)
add_subdirectory(HTWK_world_in_filter)
add_subdirectory(HTWK_world_out_filter)
add_subdirectory(HTWK_world_out_filter_base)
add_subdirectory(HTWK_world_out_template)

#Lights
add_subdirectory(HTWK_Light_Controller)

#Misc
#add_subdirectory(HTWK_structs)
#add_subdirectory(HTWK_carconfig)

add_subdirectory(HTWK_Signal_Generator)
add_subdirectory(HTWK_State_Signal_Generator)
add_subdirectory(HTWK_Bool_Signal_Generator)
add_subdirectory(HTWK_ROI_Cutter)
add_subdirectory(HTWK_ControllerPID)
add_subdirectory(HTWK_Jury_Response)

#Braking
add_subdirectory(HTWK_Emergency_Stop)
add_subdirectory(HTWK_Us_Obstacle_Detection)
add_subdirectory(HTWK_Brake)
add_subdirectory(HTWK_ChangeSpeed)

#Behavior tree
add_subdirectory(HTWK_Behavior_Tree)

#Grid Maps
add_subdirectory(HTWK_Occ_Grid_Map)

#Classification
add_subdirectory(HTWK_Classification)

#Kür
add_subdirectory(HTWK_Light_Detector)


