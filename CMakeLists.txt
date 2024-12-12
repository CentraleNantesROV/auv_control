cmake_minimum_required(VERSION 3.5)
project(auv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

rosidl_generate_interfaces(${PROJECT_NAME} "srv/Control.srv")

if("$ENV{ROS_DISTRO}" STRLESS_EQUAL "galactic")
    set(LEGACY_IDL TRUE)    
else()
    add_definitions(-DAUV_CONTROL_WITH_LOOKUP_VELOCITY)
    set(LEGACY_IDL FALSE)
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

# Decoupled cascaded PID
add_executable(cascaded_pid src/cascaded_pid.cpp src/pos_vel_pid.cpp)
target_link_libraries(cascaded_pid control_common)

# Sliding mode control
add_executable(sliding_mode src/sliding_mode.cpp)
target_link_libraries(sliding_mode control_common)

#if(NOT WIN32)
  #ament_environment_hooks(
    #"${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    #)
#endif()

install(TARGETS control_common cascaded_pid sliding_mode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# allow using the class in another package
# legacy include dir + nested since Humble
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(control_common)
ament_export_dependencies(rclcpp nav_msgs sensor_msgs tf2_ros geometry_msgs thruster_manager)
  
ament_package()
