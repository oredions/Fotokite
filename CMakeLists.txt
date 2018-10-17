cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)
project(Fotokite)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
file(GLOB SOURCES
    *.h
    *.hpp
    *.cpp
)
list(FILTER SOURCES EXCLUDE REGEX "main.*.cpp")
list(FILTER SOURCES EXCLUDE REGEX "Visualization.*")
foreach (FILE "" Takeoff GoToWaypoint ExecutePath Land)
    add_executable(Fotokite${FILE} ${SOURCES} main${FILE}.cpp)
    target_link_libraries(Fotokite${FILE} ${OpenCV_LIBS} pthread)
endforeach(FILE)
