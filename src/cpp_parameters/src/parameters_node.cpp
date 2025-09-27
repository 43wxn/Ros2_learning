#include <rclcpp/rclcpp.hpp>

class MiniaParam: public rclcpp::Node {
    public:
        MiniaParam(std::string name) : Node(name) {
            this->declare_parameter("my_param", "你好！");
            RCLCPP_INFO_STREAM(this->get_logger(), "参数 my_param 已设定");
            timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]()
                                             { this->timer_callback(); });
        }

    private:
        std::vector<rclcpp::Parameter> all_init_param{rclcpp::Parameter("my_param", "你好！")};
        rclcpp::TimerBase::SharedPtr timer_;
        void timer_callback () {
            std::string param = this->get_parameter("my_param").as_string();
            RCLCPP_INFO_STREAM(this->get_logger(), "my parameter is " << param);
            this->set_parameter(rclcpp::Parameter("my_param", "world"));
        }
};

int main(int args, char** argv) {
    rclcpp::init(args, argv);
    auto node = std::make_shared<MiniaParam>("cpp_parameter");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}