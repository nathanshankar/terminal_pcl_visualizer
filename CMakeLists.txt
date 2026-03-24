cmake_minimum_required(VERSION 3.8)
project(terminal_pcl_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Fetch FTXUI
include(FetchContent)
FetchContent_Declare(ftxui
  GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
  GIT_TAG v5.0.0
)
FetchContent_MakeAvailable(ftxui)

add_executable(visualizer_node 
  src/visualizer_node.cpp
  src/terminal_pcl_node.cpp
  src/visualizer.cpp
)
target_include_directories(visualizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(visualizer_node PUBLIC cxx_std_17)

ament_target_dependencies(visualizer_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

target_link_libraries(visualizer_node
  ftxui::screen
  ftxui::dom
  ftxui::component
)

install(TARGETS visualizer_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
