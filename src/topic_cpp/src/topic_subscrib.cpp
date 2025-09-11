//create a subscrib
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TopicSubscrib : public rclcpp::Node {
    public:
        TopicSubscrib(std::string name) : rclcpp::Node(name) {
            RCLCPP_INFO_STREAM(this->get_logger(), name << " 创建订阅者节点成功");
            subscriber = this->create_subscription<std_msgs::msg::String>(
                "command", 10,
                [this](const std_msgs::msg::String::SharedPtr msg) {
                  this->call_back(msg);
                });
        }

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
        //回调函数的参数默认使用SharedPtr 即智能指针，使用普通的参数会发生copy
        //当接收的数据很大时 copy会消耗很大的内存，不推荐
        //也可使用UniquePtr指针,Ros2的高性能零拷贝
        void call_back(const std_msgs::msg::String::SharedPtr msg) {
            //msg这个智能指针指向的十一个在Ros消息类型中的String对象 ，所以想要访问成员函数需要使用->data
            RCLCPP_INFO_STREAM(this->get_logger(), "收到了指令" << msg->data);
        }
};

int main(int argc,char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopicSubscrib>("topic_subscriber_01");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}