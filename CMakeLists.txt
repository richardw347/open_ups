cmake_minimum_required(VERSION 3.8)
project(open_ups)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


set(dependencies
rclcpp
rclcpp_components
rclcpp_lifecycle
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

add_executable(open_ups_node src/open_ups_node.cpp ${UPS_LIB})
target_include_directories(open_ups_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(open_ups_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(open_ups_node
  ${dependencies}
)
target_link_libraries(open_ups_node
  usb
)

install(TARGETS open_ups_node
  DESTINATION lib/${PROJECT_NAME})

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
