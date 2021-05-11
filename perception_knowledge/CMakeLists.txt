cmake_minimum_required(VERSION 3.5)
project(perception_knowledge)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(ros2_knowledge_graph REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(gb_visual_detection_3d_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  ros2_knowledge_graph
  vision_msgs
  gb_visual_detection_3d_msgs
)

include_directories(
  include
)

add_library(
  PerceptionKnowledge SHARED src/${PROJECT_NAME}/PerceptionKnowledge.cpp
)
ament_target_dependencies(PerceptionKnowledge ${dependencies})

add_executable(perception_knowledge_node src/perception_knowledge_node.cpp)
ament_target_dependencies(perception_knowledge_node ${dependencies})
target_link_libraries(perception_knowledge_node PerceptionKnowledge)

install(TARGETS
  PerceptionKnowledge
  perception_knowledge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

# ament_export_include_directories(include)
# ament_export_libraries(PerceptionKnowledge)
# ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
