cmake_minimum_required(VERSION 3.8)
project(legged_system)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add the custom cmake directory to CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

find_package(Eigen3 REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;${CMAKE_SOURCE_DIR}/sdk/lib")
set(CMAKE_INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  sensor_msgs
  custom_controller_msgs
  tf2
  yaml-cpp
  ament_index_cpp
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


include_directories(
  include
  include/legged_system
  ${Eigen3_INCLUDE_DIRS}
)

## COMPILE
add_library(
  legged_system
  SHARED
  src/LeggedSystem.cpp
  src/LeggedSystemHelperFunctions.cpp
  src/utilities.cpp
  src/AnkleSolver.cpp
  src/AnkleSolverHelperFunctions.cpp
)
target_compile_features(legged_system PUBLIC cxx_std_17)

target_include_directories(legged_system PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/legged_system>
)

ament_target_dependencies(
  legged_system
  PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)



find_package(Boost REQUIRED)

target_link_libraries(legged_system
                      PUBLIC
                      yaml-cpp
                      )


target_compile_definitions(${PROJECT_NAME} PRIVATE "legged_system_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface legged_system.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/legged_system
)

install(TARGETS legged_system
  EXPORT export_legged_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Install other directories
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
