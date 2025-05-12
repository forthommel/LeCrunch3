cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(LeCrunch3)
set(PROJECT_VERSION 1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_BUILD_TYPE Debug)
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")

set(EXT_LIBRARIES stdc++fs)

include_directories(${PROJECT_SOURCE_DIR})

file(GLOB LIBRARY_SOURCES src/*.cc)

add_library(LeCrunch3 SHARED ${LIBRARY_SOURCES})
target_link_libraries(LeCrunch3 ${EXT_LIBRARIES})

add_subdirectory(test)
