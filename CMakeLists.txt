cmake_minimum_required(VERSION 3.0.2)
project(nodelet_or_node)

find_package(catkin REQUIRED COMPONENTS bondcpp cmake_modules nodelet roscpp unique_id)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(load nodes/load.cpp)
add_dependencies(load ${catkin_EXPORTED_TARGETS})
target_link_libraries(load ${catkin_LIBRARIES})

install(TARGETS load
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)