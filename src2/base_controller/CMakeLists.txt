cmake_minimum_required(VERSION 2.8.3)
project(base_controller)

set(SERIAL_DRV_PATH "./serial-master/")

FILE(GLOB RPLIDAR_SDK_SRC 
  "${SERIAL_DRV_PATH}/src/impl/list_ports/*.cc"
  "${SERIAL_DRV_PATH}/src/impl/*.cc"
  "${SERIAL_DRV_PATH}/src/*.cc"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  tf
)

include_directories(
  include
  ${SERIAL_DRV_PATH}/include
  ${SERIAL_DRV_PATH}/src
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(base_controller_node src/base_controller.cpp src/base_controller_node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(base_controller_node ${catkin_LIBRARIES})

install(TARGETS base_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
