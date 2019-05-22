cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_STANDARD 14)

PROJECT(kiwi-detect)

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS})

add_executable( kiwi-detect kiwi-detect.cpp )

target_link_libraries( kiwi-detect ${OpenCV_LIBS} )