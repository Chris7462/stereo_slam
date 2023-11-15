cmake_minimum_required(VERSION 3.8)
project(stereo_slam)

set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(g2o REQUIRED)
find_package(GeographicLib REQUIRED)

# visual odometry library
add_library(visual_odom SHARED
  src/visual_odometry.cpp
  src/dataset.cpp
  src/camera.cpp
  src/frontend.cpp
  src/frame.cpp
  src/feature.cpp
  src/mappoint.cpp
  src/map.cpp
  src/backend.cpp
  src/viewer.cpp)

target_include_directories(visual_odom
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR})

target_link_libraries(visual_odom
  PUBLIC
    opencv_core
    opencv_features2d
    opencv_video
    Sophus::Sophus
    Eigen3::Eigen
    g2o::core
    g2o::solver_dense
    g2o::solver_csparse)

install(TARGETS visual_odom
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  COMPONENT library)

# exectable
add_executable(stereo_slam_node
  src/run_slam.cpp
  src/slam_node.cpp)

target_include_directories(stereo_slam_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(stereo_slam_node
  PUBLIC
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_eigen
    cv_bridge
    GeographicLib)

target_link_libraries(stereo_slam_node
  PUBLIC
    visual_odom)

target_compile_features(stereo_slam_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS stereo_slam_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
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
  #ament_lint_auto_find_test_dependencies()

  # unit testing (google test)
  find_package(ament_cmake_gtest REQUIRED)

  set(test_sources
      test_algorithm test_dataset test_map_point
      test_frame test_camera test_map test_viewer
      test_backend)
  foreach(test_src ${test_sources})
    ament_add_gtest(${test_src} test/${test_src}.cpp)
    target_link_libraries(${test_src} visual_odom gtest_main)
  endforeach()

endif()

ament_package()
