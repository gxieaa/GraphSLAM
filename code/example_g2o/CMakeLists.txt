
cmake_minimum_required(VERSION 2.8.3)

# For cmake custom library searches
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

FIND_PACKAGE(G2O REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(CSparse)

INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})


add_executable(simple_optimize src/simple_optimize.cpp )

target_link_libraries(simple_optimize
   ${G2O_LIBRARIES} 
)
