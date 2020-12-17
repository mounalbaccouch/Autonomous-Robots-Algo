cmake_minimum_required(VERSION 3.7)

project(Autonomous)

set(CMAKE_SRC_PATH "src/")
set(CMAKE_LIB_PATH "lib/") 

find_package(PythonLibs REQUIRED)
set(PYTHON_EXECUTABLE "/usr/bin/python3.8")
set(PYTHON_INCLUDE_DIR "/usr/include/python3.8")
set(PYTHON_LIBRARIES "/usr/lib/x86_64-linux-gnu/libpython3.8.so")

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${CMAKE_LIB_PATH}/matplotlib-cpp)

add_executable(oGridMapping ${CMAKE_SRC_PATH}/1-oGridMapping/oGridMapping.cpp)
target_link_libraries(oGridMapping ${PYTHON_LIBRARIES})

file(COPY ${CMAKE_SRC_PATH}/1-oGridMapping/poses.txt DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_SRC_PATH}/1-oGridMapping/measurement.txt DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

