cmake_minimum_required(VERSION 2.8.3)
project(rp_drive)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++14 -o3")

        #add_include_directories("/opt/ros/melodic/include")

LIST( APPEND CMAKE_MODULE_PATH ~/g2o/cmake_modules )
find_package(G2O)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  std_msgs
  genmsg
  laser_geometry
  sensor_msgs
  geometry_msgs
)

include_directories(
        "/usr/include/eigen3"
        "/usr/include/vtk-6.3"
        "/opt/ros/melodic/include"
        ${PCL_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${CHOLMOD_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
)

set(G2O_LIBS g2o_core g2o_stuff g2o_types_sba g2o_types_slam3d)


add_executable(rp_drive 
cmd_vel_test.cpp
cmd_vel_test.h 
)

target_link_libraries(rp_drive
        ${PCL_LIBRARIES}
        ${CHOLMOD_LIBRARIES}
        ${G2O_LIBS}
        ${catkin_LIBRARIES}
)
