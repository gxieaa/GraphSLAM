cmake_minimum_required(VERSION 2.8.3)
project(My_SLAM CXX)

# C++11 support
IF(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

FIND_PACKAGE(G2O REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(CSparse)

INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

ADD_LIBRARY(my_slam_lib ${G2O_LIB_TYPE}
  src/data_association.h src/data_association.cpp
)

SET_TARGET_PROPERTIES(my_slam_lib PROPERTIES OUTPUT_NAME ${LIB_PREFIX}my_slam)

TARGET_LINK_LIBRARIES(my_slam_lib core solver_csparse)
TARGET_LINK_LIBRARIES(my_slam_lib types_slam2d)
TARGET_LINK_LIBRARIES(my_slam_lib types_slam3d)

ADD_EXECUTABLE(my_slam src/my_slam.cpp)

SET_TARGET_PROPERTIES(my_slam PROPERTIES OUTPUT_NAME my_slam${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(my_slam core solver_csparse)
TARGET_LINK_LIBRARIES(my_slam types_slam2d)
TARGET_LINK_LIBRARIES(my_slam types_slam3d)
TARGET_LINK_LIBRARIES(my_slam my_slam_lib)
