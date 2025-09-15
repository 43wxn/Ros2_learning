#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class Server : public rclcpp::Node {
    public:
        Server(std::string name):Node(name) {
            RCLCPP_INFO_STREAM(this->get_logger(), name << " server has build");
            add_server =
                this->create_service<example_interfaces::srv::AddTwoInts>(
                    "add_two_ints",
                    [this](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                           std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
                        this->handle_add_two_ints(request,response);
                    });
        }

    private:
        //这里给Service传入的接口参数 是AddTwoInts是以为在包含的头文件add_two_ints.hpp中 自动生成了一个AddTwoInts类
        //我们需要使用的是这个类 而不是这个文件
        //文件名的命名规则是全部小写加下划线 
        //类名的命名规则是驼峰式
        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr add_server;
        void handle_add_two_ints(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request ,
                                std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response){
            response->sum = request->a + request->b;
        }
};

int main(int argc,char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Server>("service_server");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}