cmake_minimum_required(VERSION 2.8.3)
project(ndt_ed_matching)

set(CMAKE_CXX_FLAGS "-march=native -O2 -std=c++11 -fopenmp")
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS   "-march=native -O2 -fopenmp")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "-march=native -O2 -fopenmp")
set(CMAKE_EXE_LINKER_FLAGS "-march=native -O2 -fopenmp")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  pcl_ros
  pcl_conversions
  pcl_msgs
)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs
    tf
    pcl_ros
    pcl_conversions
    pcl_msgs
    INCLUDE_DIRS include
    # LIBRARIES ndt_ed_map
)

include_directories(
  include/ndt_ed_matching
  include/ndt_ed_matching/map
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

# add_library(ndt_ed_map
#   src/ndt_ed_matching/map/map.cpp)
# target_link_libraries(ndt_ed_map)

add_executable(ndt_ed_matching
  src/ndt_ed_matching.cpp
)

target_link_libraries(ndt_ed_matching
# ndt_ed_map
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS
  ndt_ed_matching
  # ndt_ed_map
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
  include/ndt_ed_matching/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
