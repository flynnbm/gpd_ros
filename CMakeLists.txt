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
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED) 
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_library(GPD_LIB NAMES gpd PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudIndexed.msg"
  "msg/CloudSamples.msg"
  "msg/CloudSources.msg"
  "msg/GraspConfig.msg"
  "msg/GraspConfigList.msg"
  "msg/SamplesMsg.msg"
  "srv/DetectGrasps.srv"
  "srv/ComputeGraspPoses.srv"
  DEPENDENCIES 
    geometry_msgs 
    std_msgs  
    sensor_msgs
)

# Build grasp_messages library
add_library(grasp_messages SHARED
  src/gpd_ros/grasp_messages.cpp
)
target_include_directories(grasp_messages
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
# Ensure message headers/typesupport exist before building the library
rosidl_target_interfaces(grasp_messages ${PROJECT_NAME} "rosidl_typesupport_cpp")

# rosidl_get_typesupport_target(grasp_messages ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(grasp_messages ${cpp_typesupport_target})

# Build grasp_plotter library
add_library(grasp_plotter SHARED
  src/gpd_ros/grasp_plotter.cpp
)
target_include_directories(grasp_plotter
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(grasp_plotter
    rclcpp
    std_msgs
    sensor_msgs
    visualization_msgs
)
# Eigen is a plain CMake package: link its IMPORTED target
target_link_libraries(grasp_plotter Eigen3::Eigen)

# Add grasp_detection_server executable
add_executable(grasp_detection_server
  src/gpd_ros/grasp_detection_server.cpp
)
target_include_directories(grasp_detection_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(grasp_detection_server
    grasp_messages
    grasp_plotter
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${GPD_LIB}
)
ament_target_dependencies(grasp_detection_server
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_eigen
  pcl_conversions
)
# Uses generated srv/msg types
rosidl_target_interfaces(grasp_detection_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(grasp_pose_server 
  src/gpd_ros/grasp_pose_server.cpp
)
ament_target_dependencies(grasp_pose_server
  geometry_msgs
  tf2_ros
  tf2_eigen
)
target_link_libraries(grasp_pose_server
    Eigen3::Eigen
)
rosidl_target_interfaces(grasp_pose_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets and headers
install(TARGETS 
  grasp_messages 
  grasp_plotter 
  grasp_detection_server
  grasp_pose_server
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

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

# Export for downstreams (make targets discoverable by other packages)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()