INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

ADD_LIBRARY(my_optimize_lib ${G2O_LIB_TYPE}
  data_association.h data_association.cpp
)

SET_TARGET_PROPERTIES(my_optimize_lib PROPERTIES OUTPUT_NAME ${LIB_PREFIX}my_optimize)

TARGET_LINK_LIBRARIES(my_optimize_lib core solver_csparse)
TARGET_LINK_LIBRARIES(my_optimize_lib types_slam2d)
TARGET_LINK_LIBRARIES(my_optimize_lib types_slam3d)

ADD_EXECUTABLE(my_slam
  my_slam.cpp
)

SET_TARGET_PROPERTIES(my_slam PROPERTIES OUTPUT_NAME my_slam${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(my_slam core solver_csparse)
TARGET_LINK_LIBRARIES(my_slam types_slam2d)
TARGET_LINK_LIBRARIES(my_slam types_slam3d)
TARGET_LINK_LIBRARIES(my_slam my_optimize_lib)
