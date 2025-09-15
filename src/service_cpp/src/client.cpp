#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class Client : public rclcpp::Node {
    public:
        Client(std::string name):Node(name) {
            RCLCPP_INFO_STREAM(this->get_logger(), name << " 客户端已运行");
            //这里的服务端和客户端的名字必须相同 这样才能服务端和客户端匹配
            add_client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        }

        void call_service(int a,int b) {
            //循环等待服务端上线
            while (!add_client->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_INFO_STREAM(this->get_logger(), "等待服务的过程被打断");
                    return;
                }
                RCLCPP_INFO_STREAM(this->get_logger(), "等待服务上线...");
            }
            //构造请求
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts_Request>();
            request->a = a;
            request->b = b;
            //调用服务端
            add_client->async_send_request(
                request,
                [this](rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture result_future){ 
                    this->result_back(result_future);
                });
        }

    private:
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr add_client;
        void result_back(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture result_future) {
            auto response = result_future.get();
            RCLCPP_INFO_STREAM(this->get_logger(), "计算结果是 " << response->sum);
        }
};

int main(int argc,char** argv) {
    rclcpp::init(argc,argv);
    auto node=std::make_shared<Client>("service_client");
    node->call_service(3,4);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}