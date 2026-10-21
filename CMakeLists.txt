cmake_minimum_required(VERSION 3.16)
project(lite_ik_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra)

if(EXISTS "/usr/bin/python3")
  set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Python interpreter" FORCE)
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Python interpreter" FORCE)
endif()

set(CMAKE_IGNORE_PATH
  "${CMAKE_IGNORE_PATH}"
  "$ENV{HOME}/anaconda3;$ENV{HOME}/anaconda3/lib;$ENV{HOME}/anaconda3/lib/cmake"
  "$ENV{HOME}/miniconda3;$ENV{HOME}/miniconda3/lib;$ENV{HOME}/miniconda3/lib/cmake")
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_IGNORE_PATH
    "$ENV{CONDA_PREFIX}" "$ENV{CONDA_PREFIX}/lib" "$ENV{CONDA_PREFIX}/lib/cmake")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(move_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(CURL REQUIRED)

find_package(yaml-cpp QUIET CONFIG
  PATHS /usr/lib/x86_64-linux-gnu/cmake/yaml-cpp NO_DEFAULT_PATH)
if(NOT yaml-cpp_FOUND)
  find_package(yaml-cpp REQUIRED)
endif()

add_library(lite_ik_node_types INTERFACE)
target_include_directories(lite_ik_node_types INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(lite_ik_node_kinematics STATIC
  src/ik_kdl.cpp
  src/ik_moveit.cpp)
target_include_directories(lite_ik_node_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(lite_ik_node_kinematics
  rclcpp Eigen3 orocos_kdl_vendor kdl_parser moveit_core moveit_msgs
  pluginlib srdfdom urdf)
target_link_libraries(lite_ik_node_kinematics yaml-cpp::yaml-cpp orocos-kdl CURL::libcurl)

add_executable(lite_ik_node src/ik_node.cpp)
target_link_libraries(lite_ik_node lite_ik_node_kinematics)
ament_target_dependencies(lite_ik_node
  rclcpp geometry_msgs sensor_msgs visualization_msgs move_msgs urdf ament_index_cpp)

add_executable(test_ik_orchestrator test/test_ik_orchestrator.cpp)
target_link_libraries(test_ik_orchestrator lite_ik_node_kinematics)

add_executable(test_ik_kdl test/test_ik_kdl.cpp)
target_link_libraries(test_ik_kdl lite_ik_node_kinematics)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(TARGETS lite_ik_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS lite_ik_node_types lite_ik_node_kinematics
  EXPORT export_lite_ik_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)
install(EXPORT export_lite_ik_node
  NAMESPACE lite_ik_node::
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_targets(export_lite_ik_node HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp Eigen3 orocos_kdl_vendor kdl_parser moveit_core moveit_msgs
  pluginlib srdfdom urdf yaml-cpp CURL)
ament_package()
