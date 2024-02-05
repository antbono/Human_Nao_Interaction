cmake_minimum_required(VERSION 3.8)
project(hri_audio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sound_play REQUIRED)
find_package(hri_interfaces REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(nao_lola_command_msgs REQUIRED)


# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(record2file src/record2file.cpp)
ament_target_dependencies(record2file rclcpp std_msgs std_srvs)   

add_executable(test_rec src/test_rec.cpp)
ament_target_dependencies(test_rec rclcpp std_srvs)   

# Install Cpp executables
install(TARGETS
  record2file
  test_rec
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/test_audio.py
  scripts/gstt_service.py
  scripts/gstt_client.py
  scripts/gtts_service.py
  scripts/gtts_client.py
  #scripts/chat_node.py
  scripts/chat_move_node.py
  scripts/test_action.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
