cmake_minimum_required(VERSION 3.8)
project(stereo_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# visual odometry library
add_library(visual_odom SHARED
  src/visual_odometry.cpp)

target_include_directories(visual_odom
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS visual_odom
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  COMPONENT library)
  #DESTINATION lib/${PROJECT_NAME})

# exectable
add_executable(stereo_slam_node
  src/run_slam.cpp
  src/slam_node.cpp)

target_include_directories(stereo_slam_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR})

ament_target_dependencies(stereo_slam_node
  PUBLIC
    rclcpp
    sensor_msgs
    cv_bridge)

target_link_libraries(stereo_slam_node
  PUBLIC
    opencv_core
    visual_odom)

target_compile_features(stereo_slam_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS stereo_slam_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})

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