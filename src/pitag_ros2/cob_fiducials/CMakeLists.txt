cmake_minimum_required(VERSION 3.8)
project(cob_fiducials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# The bundled aruco library uses deprecated throw() specs (C++03 style).
# Suppress this error so we can compile with C++17.
add_compile_options(-Wno-dynamic-exception-spec)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cob_object_detection_msgs REQUIRED)
find_package(cob_vision_utils REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

# TinyXML via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML REQUIRED tinyxml)

# Core fiducials library (no ROS dependencies)
set(project_CPP_FILES
  common/src/AbstractFiducialModel.cpp
  common/src/pi/FiducialModelPi.cpp
  common/src/aruco/arucofidmarkers.cpp
  common/src/aruco/board.cpp
  common/src/aruco/boarddetector.cpp
  common/src/aruco/cameraparameters.cpp
  common/src/aruco/cvdrawingutils.cpp
  common/src/aruco/FiducialModelAruco.cpp
  common/src/aruco/marker.cpp
  common/src/aruco/markerdetector.cpp
)

add_library(${PROJECT_NAME}_lib SHARED
  ${project_CPP_FILES}
)

target_compile_definitions(${PROJECT_NAME}_lib PRIVATE
  __LINUX__
  BOOST_FILESYSTEM_VERSION=3
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${TINYXML_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  cob_vision_utils
)

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
  ${TINYXML_LIBRARIES}
  ${Boost_LIBRARIES}
)

# ROS2 node
add_executable(fiducials
  ros/src/fiducials.cpp
)

target_compile_definitions(fiducials PRIVATE
  __LINUX__
  BOOST_FILESYSTEM_VERSION=3
)

target_include_directories(fiducials PRIVATE
  common/include
  ${OpenCV_INCLUDE_DIRS}
  ${TINYXML_INCLUDE_DIRS}
)

ament_target_dependencies(fiducials
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  geometry_msgs
  cob_object_detection_msgs
  cob_vision_utils
)

target_link_libraries(fiducials
  ${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
  ${TINYXML_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_lib fiducials
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY common/include/
  DESTINATION include
)

install(DIRECTORY common/files/
  DESTINATION share/${PROJECT_NAME}/common/files
)

install(DIRECTORY ros/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Python nodes (installed as executables so `ros2 run cob_fiducials <name>` works)
install(PROGRAMS
  ros/src/yuyv_to_mono8.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yuyv_to_mono8
)
install(PROGRAMS
  ros/scripts/evaluate_accuracy.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME evaluate_accuracy
)

ament_package()
