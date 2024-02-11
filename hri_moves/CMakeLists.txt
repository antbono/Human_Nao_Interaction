cmake_minimum_required(VERSION 3.8)
project(hri_moves)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hri_interfaces REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(nao_lola_command_msgs REQUIRED)
find_package(nao_pos_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)


# Include Cpp "include" directory
include_directories(include)

#################### JOINTS_PLAY_ACTION_SERVER ###################
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

#################### JOINTS_PLAY_ACTION_CLIENT #####################
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


#################### HEAD_TRACK_SERVER ###########################
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


#################### HEAD_TRACK_CLIENT ###########################
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


#################### LED SERVER ###########################
add_library(led_server SHARED
  src/led_action_server.cpp)

target_include_directories(led_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(led_server
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")

rclcpp_components_register_node(led_server
  PLUGIN "hri_led_action_server::LedsPlayActionServer" 
  EXECUTABLE led_action_server
)

set(LED_SERVER_INCLUDE_DEPENDS
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nao_lola_command_msgs"
  "nao_lola_sensor_msgs"
  "hri_interfaces"
  "std_msgs")

ament_target_dependencies(led_server ${LED_SERVER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${LED_SERVER_INCLUDE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS led_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

#################### LED CLIENT ###########################
add_library(led_client SHARED
  src/led_action_client.cpp
)
target_include_directories(led_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(led_client
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL"
)
ament_target_dependencies(led_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nao_lola_command_msgs"
  "nao_lola_sensor_msgs"
  "hri_interfaces"
  "std_msgs"
)
rclcpp_components_register_node(led_client
  PLUGIN "hri_led_action_client::LedsPlayActionClient" 
  EXECUTABLE led_action_client
)
install(TARGETS
  led_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#################### CHAT ACTION SERVER ###########################
add_library(chat_server SHARED
  src/chat_action_server.cpp)

target_include_directories(chat_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(chat_server
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")

rclcpp_components_register_node(chat_server
  PLUGIN "hri_chat_action_server::ChatActionServer" 
  EXECUTABLE chat_action_server
)
set(CHAT_SERVER_INCLUDE_DEPENDS
  "hri_interfaces"
  "nao_lola_command_msgs"
  "nao_lola_sensor_msgs"  
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs"
  "std_srvs")

ament_target_dependencies(chat_server ${CHAT_SERVER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${CHAT_SERVER_INCLUDE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS chat_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

#################### TEST ###############################
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
