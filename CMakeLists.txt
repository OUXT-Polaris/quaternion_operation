cmake_minimum_required(VERSION 3.5)
project(quaternion_operation)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(	
  include	
  ${EIGEN3_INCLUDE_DIR}	
)

ament_auto_add_library(${PROJECT_NAME} SHARED src/quaternion_operation.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
