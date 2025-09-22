cmake_minimum_required(VERSION 3.10)
project(gpd_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED) 
find_package(Eigen3 REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudIndexed.msg"
  "msg/CloudSamples.msg"
  "msg/CloudSources.msg"
  "msg/GraspConfig.msg"
  "msg/GraspConfigList.msg"
  "msg/SamplesMsg.msg"
  "srv/DetectGrasps.srv"
  DEPENDENCIES geometry_msgs std_msgs  sensor_msgs
)

# Build custom libraries
add_library(grasp_messages SHARED
  src/gpd_ros/grasp_messages.cpp
)
add_library(grasp_plotter SHARED
  src/gpd_ros/grasp_plotter.cpp
)

# Make your public headers available to dependents
target_include_directories(grasp_messages
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(grasp_plotter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(grasp_messages
    std_msgs
    geometry_msgs
    sensor_msgs
    tf2_eigen
)
ament_target_dependencies(grasp_plotter
    rclcpp
    std_msgs
    sensor_msgs
    visualization_msgs
)

# Eigen is a plain CMake package: link its IMPORTED target
target_link_libraries(grasp_plotter Eigen3::Eigen)

# Ensure message headers/typesupport exist before building the library
rosidl_target_interfaces(grasp_messages ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install and export libraries
install(TARGETS grasp_messages
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS grasp_plotter
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# No libraries or executables to build or install.
ament_package()