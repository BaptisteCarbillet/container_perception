cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(container_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED) 
find_package(rs_cloud REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolygonArray.msg"
  "srv/LocalizeLegs.srv"
  DEPENDENCIES geometry_msgs
)


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

# include dirs
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(plane_detector_node
  src/plane_detector.cpp
  src/plane_detector_node.cpp
)

add_executable(leg_localizer_node
  src/leg_localizer.cpp
  src/leg_localizer_node.cpp
)

add_executable(container_tracker_node
  src/container_tracker.cpp
  src/container_tracker_node.cpp
)

add_executable(leg_detector_node
  src/leg_detector.cpp
  src/leg_detector_node.cpp
)

add_executable(leg_inspector_node
  src/leg_inspector_node.cpp
)


ament_target_dependencies(plane_detector_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  pcl_conversions

  
)


ament_target_dependencies(leg_detector_node
  rclcpp
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  pcl_conversions
  
)

ament_target_dependencies(leg_localizer_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
)

ament_target_dependencies(leg_inspector_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  #std_srvs
  tf2
  tf2_geometry_msgs
  sensor_msgs
  rs_cloud
  rclcpp_lifecycle
)



ament_target_dependencies(container_tracker_node
  rclcpp
  geometry_msgs
  visualization_msgs
  std_msgs
  # tf2
  # tf2_ros
  # tf2_geometry_msgs
)

#rosidl_target_interfaces(container_tracker_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
#rosidl_target_interfaces(leg_detector_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(plane_detector_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  
)

target_link_libraries(container_tracker_node
  ${OpenCV_LIBRARIES}
  ${cpp_typesupport_target}
)

target_link_libraries(leg_detector_node
  ${PCL_LIBRARIES}
  ${cpp_typesupport_target}
)

target_link_libraries(leg_localizer_node
  ${PCL_LIBRARIES}
  ${cpp_typesupport_target}
)

target_link_libraries(leg_inspector_node
  ${cpp_typesupport_target}
)

install(TARGETS
  leg_inspector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  container_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS leg_localizer_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS plane_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS leg_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)



ament_export_dependencies(rosidl_default_runtime geometry_msgs)
ament_package()
