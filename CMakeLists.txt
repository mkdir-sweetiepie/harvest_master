cmake_minimum_required(VERSION 3.8)
project(harvest_master)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Qt5 dependencies
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Qt5 settings
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Include directories
include_directories(include)

# Set Qt5 files
set(QT_SOURCES
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
)

set(QT_HEADERS
  include/harvest_master/main_window.hpp
  include/harvest_master/qnode.hpp
)

set(QT_FORMS
  ui/mainwindow.ui
)

set(QT_RESOURCES
  resources/images.qrc
)

# Qt5 wrap files
qt5_wrap_cpp(QT_HEADERS_MOC ${QT_HEADERS})
qt5_wrap_ui(QT_FORMS_HEADERS ${QT_FORMS})
qt5_add_resources(QT_RESOURCES_RCC ${QT_RESOURCES})

# Add executable
add_executable(${PROJECT_NAME}
  ${QT_SOURCES}
  ${QT_HEADERS_MOC}
  ${QT_FORMS_HEADERS}
  ${QT_RESOURCES_RCC}
)

# Target include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link Qt5 libraries
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
)

# ROS 2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
