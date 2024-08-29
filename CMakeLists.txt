cmake_minimum_required(VERSION 2.8.3)
project(eskf_iwo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb")
set(CMAKE_CXX_FLAGS_DEBUG "-g")

add_definitions("-DCATKIN_ENABLE_TESTING=0")

find_package(OpenCV 4.5.1 QUIET COMPONENTS imgproc highgui)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  sprayerone_msgs
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf 
  DEPENDS OpenCV EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(eskf_iwo
  src/run_eskf_gins_wjx.cc
  src/static_imu_init.cc
)

add_dependencies(eskf_iwo
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(eskf_iwo
  ${catkin_LIBRARIES}
  glog
  yaml-cpp
  gflags
)

add_library(eskf_iwo_nodelet
  src/eskf_iwo_nodelet.cpp
)
add_dependencies(eskf_iwo_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(eskf_iwo_nodelet
  eskf_iwo
  ${catkin_LIBRARIES}
)

install(TARGETS
  eskf_iwo eskf_iwo_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*_nodelet.h" EXCLUDE
)

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
