#include "rclcpp/rclcpp.hpp"
// 在自定义消息类型时 std_msgs改为自定义的消息功能包
// msg为固定写法，固定目录，ROS 2 规定所有消息文件 .msg 编译后，都会被放到
// <包名>/msg/ 下面。
//string.hpp是消息名称 改为自己定义的消息名称.hpp（小写）
#include "std_msgs/msg/string.hpp"

class TopicPublisher:public rclcpp::Node {
    public:
        TopicPublisher(std::string name):Node(name) {
            RCLCPP_INFO_STREAM(this->get_logger(), name << " 节点已创建");
            command_publisher = this->create_publisher<std_msgs::msg::String>("command", 10);
            timer = this->create_wall_timer(std::chrono::milliseconds(500), [this]()
                                            { this->timer_callback(); });//发布者的回调函数用来发布数据
        }

    private:
        void timer_callback() {
            //创建消息
            std_msgs::msg::String message;
            message.data = "you are handsome!";
            command_publisher->publish(message);
        }
        //command_publisher是SharePtr类型 这个类型在rclcpp的Publisher模板中
        //必须传入消息类型
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher;
        rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc,char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopicPublisher>("topic_publisher_01");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}