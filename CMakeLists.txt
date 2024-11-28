cmake_minimum_required(VERSION 3.8)
project(open_ups)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)


set(dependencies
rclcpp
rclcpp_components
sensor_msgs
)

include_directories(
  include
  src/lib
  src/utils
)

set(UPS_LIB
  src/lib/usbhid.cpp
  src/lib/HIDOpenUPS.cpp
  src/lib/HIDInterface.cpp
  src/utils/util.cpp
  src/utils/HArray.cpp
)

# add_library(ups_lib SHARED ${UPS_LIB})
# target_link_libraries(ups_lib
#   usb
# )

add_library(open_ups SHARED
  src/open_ups_node.cpp ${UPS_LIB}
)

target_include_directories(open_ups PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(open_ups
  PLUGIN "open_ups::OpenUPSNode"
  EXECUTABLE open_ups_node
)
ament_target_dependencies(open_ups
  ${dependencies}
)
target_link_libraries(open_ups
  usb
)

# add_executable(open_ups_node src/open_ups_node.cpp ${UPS_LIB})
# target_include_directories(open_ups_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(open_ups_node PUBLIC c_std_99 cxx_std_17)
# ament_target_dependencies(open_ups_node
#   ${dependencies}
# )
install(
  TARGETS open_ups ${plugin_libs}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
