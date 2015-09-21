INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

ADD_LIBRARY(my_optimize_lib ${G2O_LIB_TYPE}
  data_association.h data_association.cpp
)

SET_TARGET_PROPERTIES(my_optimize_lib PROPERTIES OUTPUT_NAME ${LIB_PREFIX}my_optimize)

TARGET_LINK_LIBRARIES(my_optimize_lib core solver_csparse)

ADD_EXECUTABLE(my_optimize
  my_optimize.cpp
)

SET_TARGET_PROPERTIES(my_optimize PROPERTIES OUTPUT_NAME my_optimize${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(my_optimize core solver_csparse)
TARGET_LINK_LIBRARIES(my_optimize types_slam2d)
TARGET_LINK_LIBRARIES(my_optimize types_slam3d)
TARGET_LINK_LIBRARIES(my_optimize my_optimize_lib)
