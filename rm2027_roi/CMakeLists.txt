cmake_minimum_required(VERSION 3.8)
project(rm2027_roi)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoiDetection.msg"
  "msg/RoiEstimate.msg"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(rm2027_roi_core
  src/core/roi_pipeline.cpp
  src/detection/rg_detector.cpp
  src/estimation/kalman_estimator.cpp
  src/coding/crop_qpmap.cpp
  src/coding/packetizer.cpp
)
target_include_directories(rm2027_roi_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rm2027_roi_core PUBLIC ${OpenCV_LIBS} Eigen3::Eigen)

add_executable(roi_detector_node src/ros/roi_detector_node.cpp)
ament_target_dependencies(roi_detector_node
  rclcpp sensor_msgs cv_bridge)
target_link_libraries(roi_detector_node rm2027_roi_core "${cpp_typesupport_target}")

add_executable(roi_estimator_node src/ros/roi_estimator_node.cpp)
ament_target_dependencies(roi_estimator_node
  rclcpp std_msgs)
target_link_libraries(roi_estimator_node rm2027_roi_core "${cpp_typesupport_target}")

add_executable(roi_crop_node src/ros/roi_crop_node.cpp)
ament_target_dependencies(roi_crop_node
  rclcpp sensor_msgs std_msgs cv_bridge)
target_link_libraries(roi_crop_node rm2027_roi_core "${cpp_typesupport_target}")

add_executable(offline_demo src/main.cpp)
target_link_libraries(offline_demo rm2027_roi_core)

install(TARGETS
  rm2027_roi_core
  roi_detector_node
  roi_estimator_node
  roi_crop_node
  offline_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kalman test/test_kalman.cpp)
  target_link_libraries(test_kalman rm2027_roi_core)
  ament_add_gtest(test_coding test/test_coding.cpp)
  target_link_libraries(test_coding rm2027_roi_core)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry rm2027_roi_core)
  ament_add_gtest(test_pipeline test/test_pipeline.cpp)
  target_link_libraries(test_pipeline rm2027_roi_core)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
