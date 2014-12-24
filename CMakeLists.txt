cmake_minimum_required(VERSION 2.8)

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

SET(CMAKE_BUILD_TYPE Release)

# Declare QUICKBSP project.
project(QUICKBSP CXX C)

## Look for Google Protocol Buffers library.
include(FindProtobuf)
find_package(Protobuf REQUIRED)
find_package(Threads REQUIRED)

# Look for flags to enable C++11 support, if any.
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_HAS_STD_CXX11)
if (COMPILER_HAS_STD_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_HAS_STD_CXX0X)
    if (COMPILER_HAS_STD_CXX0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    endif()
endif()

# Include path for TMB and Protobuf headers.
include_directories(include/)

# TMB
add_subdirectory(../../tmb/trunk ${PROJECT_BINARY_DIR}/tmb)
include_directories(../../tmb/trunk/include)

add_executable(
  shortest_path    
  src/examples/shortest_path.cc
  )


add_executable(
  pagerank
  src/examples/pagerank.cc
  )

add_executable(
  connected_component
  src/examples/connected_component.cc
  )

add_library(
  bsp 
  src/bsp.cc
  )

target_link_libraries(
  bsp
  ${CMAKE_THREAD_LIBS_INIT}
  tcmalloc
  ${PROTOBUF_LIBRARY}
  tmb
  )

target_link_libraries(
  shortest_path
  bsp
  )

target_link_libraries(
  pagerank
  bsp
  )

target_link_libraries(
  connected_component
  bsp
  )

# Test
#include_directories(third_party/gtest/include)
#enable_testing()

#add_executable(vertex_layer_unittest
#               tests/vertex_layer_unittest.cc)
#target_link_libraries(vertex_layer_unittest
#                      gtest
#                      gtest_main
#                      bsp)
#add_test(vertex_layer_unittest vertex_layer_unittest)
