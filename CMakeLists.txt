cmake_minimum_required(VERSION 3.5)
project(auv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(thruster_manager REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

rosidl_generate_interfaces(${PROJECT_NAME} "srv/Control.srv")

if("$ENV{ROS_DISTRO}" STREQUAL "galactic" OR "$ENV{ROS_DISTRO}" STREQUAL "foxy")
    set(LEGACY_IDL TRUE)
else()
    set(LEGACY_IDL FALSE)
endif()
if(NOT ${LEGACY_IDL})
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
endif()

# common lib, low-level control + model parsing
add_library(control_common SHARED src/controller_io.cpp src/hydrodynamics.cpp)
target_include_directories(control_common PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(control_common rclcpp urdf thruster_manager sensor_msgs geometry_msgs nav_msgs tf2_ros)
if(${LEGACY_IDL})
    rosidl_target_interfaces(control_common ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    target_link_libraries(control_common "${cpp_typesupport_target}")
endif()

# Body cascaded PID
add_executable(body_pid src/body_pid.cpp)
target_link_libraries(body_pid control_common)

# Body sliding mode
add_executable(body_sm src/body_sm.cpp)
target_link_libraries(body_sm control_common)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

install(TARGETS control_common body_pid body_sm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION share/${PROJECT_NAME})
  
ament_package()
