cmake_minimum_required(VERSION 3.8)
project(slamcore)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(g2o REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

set(COMMON_LIBS
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  cv_bridge
  image_transport
  pcl_ros
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${G2O_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  Eigen3::Eigen
)

add_library(slam_core src/frontend_lidar.cpp
                      src/frontend_visual.cpp
                      src/imu_integration.cpp
                      src/backend_optimizer.cpp
                      src/loop_closure.cpp
                      src/map_builder.cpp
                      src/frame.cpp)

target_include_directories(slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(slam_core PUBLIC ${COMMON_LIBS})

add_executable(lidar_odometry_node src/nodes/lidar_odometry_node.cpp)
target_link_libraries(lidar_odometry_node slam_core ${COMMON_LIBS})

add_executable(visual_odometry_node src/nodes/visual_odometry_node.cpp)
target_link_libraries(visual_odometry_node slam_core ${COMMON_LIBS})

add_executable(imu_integration_node src/nodes/imu_integration_node.cpp)
target_link_libraries(imu_integration_node slam_core ${COMMON_LIBS})

add_executable(backend_optimizer_node src/nodes/backend_optimizer_node.cpp)
target_link_libraries(backend_optimizer_node slam_core ${COMMON_LIBS})

add_executable(loop_closure_node src/nodes/loop_closure_node.cpp)
target_link_libraries(loop_closure_node slam_core ${COMMON_LIBS})

add_executable(map_builder_node src/nodes/map_builder_node.cpp)
target_link_libraries(map_builder_node slam_core ${COMMON_LIBS})

install(TARGETS
  lidar_odometry_node
  visual_odometry_node
  imu_integration_node
  backend_optimizer_node
  loop_closure_node
  map_builder_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY scripts/ DESTINATION lib/${PROJECT_NAME}/scripts)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_gtest(test_frame_node tests/test_frame.cpp)
  target_link_libraries(test_frame_node slam_core ${COMMON_LIBS})
endif()

ament_package()
