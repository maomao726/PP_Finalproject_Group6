cmake_minimum_required(VERSION 3.1.2)
project(detect)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenMP REQUIRED)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

file(GLOB SRC_FILES *.cpp)
add_executable(${CMAKE_PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${CMAKE_PROJECT_NAME} CONAN_PKG::opencv OpenMP::OpenMP_CXX)