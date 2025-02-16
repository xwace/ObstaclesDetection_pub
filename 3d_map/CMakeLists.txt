cmake_minimum_required(VERSION 2.6)
project(pcl_test)

set(WITH_VISUAL ON)
# set(WITH_VISUAL OFF)

# set(FLY_FILE ON)
set(FLY_FILE OFF)


set(CMAKE_CXX_STANDARD 11)

if (${WITH_VISUAL} STREQUAL ON)
        add_definitions(-DVISUAL)
endif()

if (${FLY_FILE} STREQUAL ON)
        add_definitions(-DPLY_FILE_ON)
endif()


# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#子模块
add_subdirectory(devices)
add_subdirectory(process)
add_subdirectory(common)


set(OTHER_LIBRARIES common  devices process)
include_directories(${PROJECT_SOURCE_DIR}/devices)
include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/process)
link_directories(${PROJECT_SOURCE_DIR}/build/lib)
# add_definitions(-g)

add_executable(main main.cpp)
target_link_libraries (main
                ${OTHER_LIBRARIES}
                ${PCL_LIBRARIES}
                pthread
                )