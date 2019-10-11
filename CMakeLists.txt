cmake_minimum_required(VERSION 3.5)
project(quaternion_operation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(quaternion_operation src/quaternion_operation.cpp)
ament_target_dependencies(quaternion_operation rclcpp geometry_msgs tf2_ros)

# install executables/libs
install(TARGETS
  quaternion_operation
  DESTINATION lib/quaternion_operation
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

#if (CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest_gtest(test-quaternion_operation
#  test/test_quaternion_operation.test
#  test/src/test_quaternion_operation.cpp)
#  target_link_libraries(test-quaternion_operation
#  quaternion_operation
#  ${catkin_LIBRARIES})
#endif ()

ament_package()