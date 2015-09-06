INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

ADD_EXECUTABLE(my_optimize
  my_optimize.cpp
)

SET_TARGET_PROPERTIES(my_optimize PROPERTIES OUTPUT_NAME my_optimize${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(my_optimize core solver_csparse)
TARGET_LINK_LIBRARIES(my_optimize types_slam2d)
TARGET_LINK_LIBRARIES(my_optimize types_slam3d)
