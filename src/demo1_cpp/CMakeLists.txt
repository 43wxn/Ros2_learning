cmake_minimum_required(VERSION 3.8)
project(demo1_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

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

#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> my_node >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#add_executable(<可执行文件名称> <源文件>)  可执行文件件名字最终是生成的可执行文件放在install中的名称 不需要与节点名称一样
add_executable(node_01 src/node_01.cpp)
add_executable(node_oop_01 src/node_oop_01.cpp)
#动态编译 将编译好的node_01和rclcpp链接起来 使得node_01能使用rclcpp提供的函数
ament_target_dependencies(node_01 rclcpp) 
ament_target_dependencies(node_oop_01 rclcpp)

#表示将构建好的可执行文件方式到lib/${PROJECT_NAME} 这个路径下 
#TARGETS 表示要安装一个构建目标
# DESTINATION表示按照的路径
#node_01就是编译好的可执行文件 
#所以 node_01这个节点就会被放在install文件下的lib/demo1_cpp这个路径下 （ros的工作空间中install/lib是放置功能包的
#功能包又放置相应的节点可执行文件）
install(TARGETS node_01 DESTINATION lib/${PROJECT_NAME})
install(TARGETS node_oop_01 DESTINATION lib/${PROJECT_NAME})