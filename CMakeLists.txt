cmake_minimum_required(VERSION 3.8)
project(harvest_master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Qt5 dependencies
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Set Qt5 MOC, UIC, RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# IMPORTANT: Set the UI search path for AUTOUIC
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# Include directories
include_directories(include)

# Qt5 UI files (full path)
set(UI_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/ui/mainwindow.ui
)

# Qt5 Resource files  
set(RESOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/images.qrc
)

# Source files
set(SOURCES
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
)

# Header files that need MOC
set(HEADERS
  include/harvest_master/main_window.hpp
  include/harvest_master/qnode.hpp
)

# Create executable
add_executable(${PROJECT_NAME}
  ${SOURCES}
  ${HEADERS}
  ${UI_FILES}
  ${RESOURCE_FILES}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
  ${OpenCV_LIBRARIES}
)

# ROS2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  OpenCV
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  ui/
  DESTINATION share/${PROJECT_NAME}/ui/
)

install(DIRECTORY
  resources/
  DESTINATION share/${PROJECT_NAME}/resources/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()