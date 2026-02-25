cmake_minimum_required(VERSION 3.8)
project(rl_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(CONTROLLER_INCLUDE_DEPENDS
    pluginlib
    rcpputils
    controller_interface
    realtime_tools
    urdf
    std_msgs
    tf2_ros
)

find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

# Find ONNX Runtime
# find_package(onnxruntime REQUIRED)

foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  rl_controllers
  SHARED
  src/RLControllerBase.cpp
  src/AcController.cpp
  src/utilities.cpp
)

target_compile_features(rl_controllers PUBLIC cxx_std_17)
target_include_directories(rl_controllers PRIVATE
  ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime/include
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/rl_controllers>
)

ament_target_dependencies(
  rl_controllers
  PUBLIC
  ${CONTROLLER_INCLUDE_DEPENDS}
  # onnxruntime
)

target_link_libraries(rl_controllers
  PRIVATE
  ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime/lib/libonnxruntime.so  # 替换为实际文件名
  yaml-cpp
  # onnxruntime
)

target_compile_definitions(rl_controllers PRIVATE "RL_CONTROLLERS_BUILDING_DLL")

# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface rl_controllers.xml)

# INSTALL
install(
  DIRECTORY config launch policy pkl_data
  DESTINATION share/rl_controllers
)

install(
  DIRECTORY
    ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime/include/
  DESTINATION
    include/thirdparty/onnxruntime
)

install(
  DIRECTORY
    ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime/lib/
  DESTINATION
    lib
)

install(
  FILES rl_controllers.xml
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS rl_controllers
  EXPORT export_rl_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# EXPORTS
ament_export_targets(export_rl_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS})
ament_export_dependencies(onnxruntime)
ament_package()
