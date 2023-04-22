cmake_minimum_required(VERSION 3.25)
project(PCautoreg)

set(CMAKE_CXX_STANDARD 17)
#set(APP_DOMAIN /usr/local/include/pcl1.13)

#Include the directories and now your cpp files will recognize your headers
#include_directories(${APP_DOMAIN})

set(PCL_DIR "/usr/local/include/pcl1.13")
find_package(PCL 1.3 REQUIRED )

 #   message(FATAL_ERROR  ${PCL_INCLUDE_DIRS} )
include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(includes/LASlib/inc)
include_directories(includes/LASzip/src)

add_subdirectory(includes/LASlib/src)

add_executable(PCautoreg  main.cpp  readwrite.cpp grid_maximum.cpp   keypoints2features.cpp   alignRANSAC.h)

target_link_libraries(PCautoreg ${PCL_LIBRARIES} LASlib)