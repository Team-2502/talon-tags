cmake_minimum_required(VERSION 3.27)
project(talon_tags)

find_package(OpenCV REQUIRED)
find_package(wpilib REQUIRED)

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
link_directories( ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_STANDARD 17)

add_executable(talon_tags src/main.cpp
        src/data.cpp
        src/data.h)
TARGET_LINK_LIBRARIES(talon_tags ${OpenCV_LIBS})
target_link_libraries(talon_tags ntcore wpiutil apriltag)
