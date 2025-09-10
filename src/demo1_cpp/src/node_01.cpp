#include "rclcpp/rclcpp.hpp"


int main(int argc,char **argv) {
    //初始化rclcpp
    rclcpp::init(argc, argv);
    //用make_shared函数传入一个rclcpp::Node类 返回一个该类的智能指针 ->创建一个节点
    auto node = std::make_shared<rclcpp::Node>("node_01");
    /*
    RCLCPP_INFO是一个日志输出宏 格式为 RCLCPP_INFO(logger, format_string, ...);
    1.node->get_logger()
        每个ros节点都有一个日志器（logger）
        get_logger()返回一个logger对象（该节点的日志器）
    2.输出格式
        也可以按照c语言的printf格式来写 使用%s，%d占位符
        RCLCPP_INFO(node->get_logger(),"hello %s,number=%d","Ros",42)
        结果为hello Ros，number=42
    */
    RCLCPP_INFO(node->get_logger(), "node_01已经启动");
    //循环运行节点
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}