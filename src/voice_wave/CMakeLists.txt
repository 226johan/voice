cmake_minimum_required(VERSION 3.8)
project(voice_wave)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_library(ALSA_LIBRARY asound)

# 添加可执行目标
add_executable(voice_wave_node src/voice_wave.cpp)
ament_target_dependencies(voice_wave_node
  "rclcpp")

# 包含头文件目录
target_include_directories(voice_wave_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/voice_wave)

# 连接库目录和库文件
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)
target_link_libraries(voice_wave_node
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/libmsc.so
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/libw_ivw.so
  rclcpp::rclcpp
  ${ALSA_LIBRARY})

# 安装可执行文件到ros2的lib目录
install(TARGETS 
  voice_wave_node
  DESTINATION lib/${PROJECT_NAME})

# 安装头文件
install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME})

# 安装bin目录中的文件
install(DIRECTORY
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

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

