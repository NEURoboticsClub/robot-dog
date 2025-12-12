cmake_minimum_required(VERSION 2.8.3)
project(champ_behavior_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  champ_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES champ_behavior_core
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    champ_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(champ_behavior_core
  src/gait_profile.cpp
  src/gait_manager.cpp
  src/behavior_state_machine.cpp
)
add_dependencies(champ_behavior_core ${catkin_EXPORTED_TARGETS})

add_executable(behavior_gait_node src/behavior_gait_node.cpp)
target_link_libraries(behavior_gait_node
  champ_behavior_core
  ${catkin_LIBRARIES}
)
add_dependencies(behavior_gait_node ${catkin_EXPORTED_TARGETS})

install(TARGETS
  champ_behavior_core
  behavior_gait_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_behavior_gait_manager
    test/test_behavior_gait_manager.cpp
  )
  if(TARGET test_behavior_gait_manager)
    target_link_libraries(test_behavior_gait_manager
      champ_behavior_core
      ${catkin_LIBRARIES}
    )
  endif()
endif()


