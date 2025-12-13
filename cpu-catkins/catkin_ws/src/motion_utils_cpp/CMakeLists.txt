cmake_minimum_required(VERSION 2.8.3)
project(motion_utils_cpp)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motion_utils_core
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(motion_utils_core
  src/first_order_filter.cpp
  src/pid_controller.cpp
  src/twist_limiter.cpp
)
add_dependencies(motion_utils_core ${catkin_EXPORTED_TARGETS})

add_executable(motion_utils_node src/motion_utils_node.cpp)
target_link_libraries(motion_utils_node
  motion_utils_core
  ${catkin_LIBRARIES}
)
add_dependencies(motion_utils_node ${catkin_EXPORTED_TARGETS})

install(TARGETS
  motion_utils_core
  motion_utils_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(test_first_order_filter
    test/test_first_order_filter.cpp
  )
  if(TARGET test_first_order_filter)
    target_link_libraries(test_first_order_filter
      motion_utils_core
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_pid_controller
    test/test_pid_controller.cpp
  )
  if(TARGET test_pid_controller)
    target_link_libraries(test_pid_controller
      motion_utils_core
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_twist_limiter
    test/test_twist_limiter.cpp
  )
  if(TARGET test_twist_limiter)
    target_link_libraries(test_twist_limiter
      motion_utils_core
      ${catkin_LIBRARIES}
    )
  endif()
endif()


