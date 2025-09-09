#include "rclcpp/rclcpp.hpp"
int main(int argc,char **argv) {//必须写int argc,char **argv，节点需要解析这些参数
    rclcpp::init(argc, argv);
    // make_shared<T>(args..)作用是在堆上创建一个对象，并返回一个 std::shared_ptr<T> 智能指针
    //std::shared_ptr<T> 智能指针会自动管理内存 当引用计数归零时自动释放
    rclcpp::spin(std::make_shared<rclcpp::Node>("first_node"));
    return 0;
}