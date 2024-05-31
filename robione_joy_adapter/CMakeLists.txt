cmake_minimum_required(VERSION 3.8)
project(robione_joy_adapter_autoware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()
include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

set(TARGET_SRC
        src/robione_joy_adapter.cpp
        src/joy_autoware_cmd_converter.cpp
)

ament_auto_add_library(${PROJECT_NAME}  ${TARGET_SRC})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "robione_joy_adapter::RobioneJoyAdapter"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)


ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)