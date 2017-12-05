cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)
project(Fotokite)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/examples/)
link_directories(${CMAKE_SOURCE_DIR}/examples/)
file(GLOB SOURCES
    *.h
    *.hpp
    *.cpp
)
add_executable(Fotokite ${SOURCES})
target_link_libraries(Fotokite ${OpenCV_LIBS})