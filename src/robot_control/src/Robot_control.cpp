#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/robot_status.hpp"
#include "interfaces/srv/move_robot.hpp"

/*
该节点1.向机器人发送控制请求 2.订阅机器人状态
*/
class RobotControl:public rclcpp::Node {
    public:
        RobotControl(std::string name):Node(name) {
            RCLCPP_INFO_STREAM(this->get_logger(), "机器人控制节点已创建 " << name);
            robot_client_ = this->create_client<interfaces::srv::MoveRobot>(
                "move_robot"
            );

            status_subscriber_ = this->create_subscription<interfaces::msg::RobotStatus>(
                "robot_status",
                10,
                [this](interfaces::msg::RobotStatus::ConstSharedPtr status) {
                    this->status_callback(status);
                }
                
            );
        }

        /*
        请求服务函数
        */
        void move_robot(float distance) {
            RCLCPP_INFO_STREAM(this->get_logger(),"请求机器人移动： " << distance);
            while (!robot_client_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_INFO_STREAM(this->get_logger(), "异常中断，正在退出...");
                    return;
                }
                RCLCPP_INFO_STREAM(this->get_logger(), "等待服务器...");
            }
            auto request = std::make_shared<interfaces::srv::MoveRobot::Request>();
            request->distance=distance;
            robot_client_->async_send_request(
                request,
                [this](rclcpp::Client<interfaces::srv::MoveRobot>::SharedFuture future) {
                    this->result_callback(future);
                });
        }

    private:
        rclcpp::Client<interfaces::srv::MoveRobot>::SharedPtr robot_client_;
        rclcpp::Subscription<interfaces::msg::RobotStatus>::SharedPtr status_subscriber_;

        /*
        订阅话题消息的回调函数
        */
        void status_callback(interfaces::msg::RobotStatus::ConstSharedPtr status) {
           RCLCPP_INFO_STREAM(this->get_logger(), "机器人现在位置: " << status->pose << "状态为： " << status->status);
        }

       /*
       服务结束回调函数
       */
        void result_callback(rclcpp::Client<interfaces::srv::MoveRobot>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO_STREAM(this->get_logger(), "移动后的位置： " << response->pose);
        }
};

int main(int argc,char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControl>("robot_control");
    node->move_robot(10);//控制机器人向前走10m
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}