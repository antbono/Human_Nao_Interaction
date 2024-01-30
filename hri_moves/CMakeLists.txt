cmake_minimum_required(VERSION 3.8)
project(hri_moves)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(nao_lola_command_msgs REQUIRED)
find_package(hri_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(naosoccer_pos_action_interfaces REQUIRED)



# Include Cpp "include" directory
include_directories(include)


# Composition nodes
add_library(action_server SHARED
  src/joints_play_action_server.cpp
)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL"
)
ament_target_dependencies(action_server
  "hri_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nao_lola_command_msgs"
)
rclcpp_components_register_node( action_server 
  PLUGIN "hri_action_cpp::JointsPlayActionServer" 
    EXECUTABLE joints_play_action_server
)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)



add_library(action_client SHARED
  src/joints_play_action_client.cpp
)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(action_client
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL"
)
ament_target_dependencies(action_client
  "hri_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client 
  PLUGIN "hri_action_cpp::JointsPlayActionClient" 
    EXECUTABLE joints_play_action_client
)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


add_library(head_track_server SHARED
  src/head_track_action_server.cpp
)
target_include_directories(head_track_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(head_track_server
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL"
)
ament_target_dependencies(head_track_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nao_lola_command_msgs"
  "nao_lola_sensor_msgs"
  "hri_interfaces"
)
rclcpp_components_register_node(head_track_server 
  PLUGIN "hri_head_track_cpp::HeadTrackActionServer" 
  EXECUTABLE head_track_action_server
)
install(TARGETS
  head_track_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



add_library(head_track_client SHARED
  src/head_track_action_client.cpp
)
target_include_directories(head_track_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(head_track_client
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL"
)
ament_target_dependencies(head_track_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nao_lola_command_msgs"
  "nao_lola_sensor_msgs"
  "hri_interfaces"
)
rclcpp_components_register_node(head_track_client 
  PLUGIN "hri_head_track_cpp::HeadTrackActionClient" 
  EXECUTABLE head_track_action_client
)
install(TARGETS
  head_track_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



add_library(naosoccer_action_client SHARED
  src/naosoccer_client.cpp
)
target_include_directories(naosoccer_action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(naosoccer_action_client
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL"
)
ament_target_dependencies(naosoccer_action_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nao_lola_command_msgs"
  "nao_lola_sensor_msgs"
  "hri_interfaces"
  "naosoccer_pos_action_interfaces"
)
rclcpp_components_register_node(naosoccer_action_client 
  PLUGIN "hri_naosoccer_action_client::NaosoccerActionClient" 
  EXECUTABLE naosoccer_client
)
install(TARGETS
  naosoccer_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


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

ament_package()
