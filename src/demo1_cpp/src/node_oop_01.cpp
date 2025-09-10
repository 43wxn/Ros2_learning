#include "rclcpp/rclcpp.hpp"
class Node_oop : public rclcpp::Node {
    public:
        Node_oop(std::string name) :Node(name) {
            //这里RCLCPP_INFO是需要c风格的输入 所以 不能直接使用name输入，name是std::string 类型的 
            //而%s需要的是const char*指针 指向需要传入的字符串 
            //.c_str()返回一个const char*类型的指针 
            RCLCPP_INFO(this->get_logger(), "%s节点创建成功", name.c_str());
            //也可以使用c++20的RCLCPP_INFO_STREAM
            RCLCPP_INFO_STREAM(this->get_logger(), name << " 节点创建成功");
        }
    private:

};

int main(int argc,char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node_oop>("oop_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}