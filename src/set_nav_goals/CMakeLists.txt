cmake_minimum_required(VERSION 2.8.3)
project(set_nav_goals)
find_package(catkin REQUIRED COMPONENTS actionlib_msgs message_generation actionlib move_base_msgs roscpp tf netbase_msgs std_msgs)

add_action_files(FILES
  CtrlLamp.action
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime)

include_directories(
  include ${catkin_INCLUDE_DIRS}${catkin_INCLUDE_DIRS})

# Build the executable
# Add a build order dependency on nav_msgs
# This ensures that all msg headers are built before your executable


#生成依赖库，给其他功能包调用
add_library(control_lamp 
  include/set_nav_goals/control_lamp.h
  src/control_lamp.cpp)
add_dependencies(control_lamp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_lamp
  ${catkin_LIBRARIES}
)
#生成依赖库，给其他功能包调用



add_executable(set_goals_node
  include/set_nav_goals/moveCtrl.h
  src/moveCtrl.cpp
  include/set_nav_goals/control_lamp.h
  src/control_lamp.cpp
  include/set_nav_goals/set_goals.h
  src/set_goals.cpp
  src/set_goals_node.cpp
  src/zjucmd.cc
  src/zjucode.cc
  src/zjudeal.cc
)

add_dependencies(set_goals_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(set_goals_node
  ${catkin_LIBRARIES}
)

