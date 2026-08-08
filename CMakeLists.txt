cmake_minimum_required(VERSION 3.8)
project(my_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  Boost
  tf2_eigen
  tf2_geometry_msgs
  moveit_core
  moveit_task_constructor_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  geometric_shapes
  visualization_msgs
  interactive_markers
  rviz_visual_tools
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(mtc_node src/mtc_node.cpp)
add_executable(robot_model_state src/robot_model_state.cpp)
add_executable(motion_planning_pipeline src/motion_planning_pipeline.cpp)
add_executable(motion_planning_api src/motion_planning_api.cpp)
add_executable(display_robot src/display_robot.cpp)
add_executable(planning_scene_ros_api src/planning_scene_ros_api.cpp)
add_executable(planning_scene src/planning_scene.cpp)
add_executable(move_group_interface src/move_group_interface.cpp)

target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(mtc_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_target_dependencies(robot_model_state
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_target_dependencies(motion_planning_pipeline
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_target_dependencies(motion_planning_api
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_target_dependencies(display_robot
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_target_dependencies(planning_scene
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_target_dependencies(planning_scene_ros_api
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_target_dependencies(move_group_interface
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS 
  mtc_node
  robot_model_state
  motion_planning_api
  motion_planning_pipeline
  planning_scene
  planning_scene_ros_api
  display_robot
  move_group_interface
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
