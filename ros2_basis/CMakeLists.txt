cmake_minimum_required(VERSION 3.23)
project(ros2_basis)

find_package(rclcpp REQUIRED)   #依赖rclcpp这个包 REQUIRED表示如果没有找到则报错 编译停止

add_executable(first_node first_node.cpp)   #创建一个可执行文件 名字叫first_node 从first_node.cpp编译而来

#把创建的可执行文件和 ROS 2 库链接起来，这样节点才能使用 rclcpp::Node、rclcpp::spin 等功能。
target_link_libraries(first_node rclcpp::rclcpp)    


#find_package的查找路径：
#<package>_DIR
#CMAKE_PREFIX_PATH
#CMAKE_FRAMEWORK_PATH
#CMAKE_APPBUNDLE_PATH
#PATH
#PATH中有/opt/ros/{$DISRO}/bin
#cmake找到后，会开始依次找<package>Config.cmake或Find<package>.cmake文件。找到后即可执行该文件并生成相关链接信息