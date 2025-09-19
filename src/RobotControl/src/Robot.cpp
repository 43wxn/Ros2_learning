#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/move_robot.hpp"
#include "interfaces/msg/robot_status.hpp"

/*
Robot类模拟机器人 提供三个接口：
1.控制机器人前进一段距离 ： move_distance
2.查询机器人当然位置 ： get_cur_pose
3.查询机器人当前状态 : get_status
*/

class Robot{
    public:
        Robot() = default;
        ~Robot() = default;

        /*控制机器人前进一段距离
          @param distance
          @return float
        */
       float move_distance(float distance) {
           status_ = interfaces::msg::RobotStatus::STATUS_MOVEING;
           target_pose_ += distance;
           
           while (fabs(target_pose_-cur_pose_)<0.1) {
            //每一步都移动当前位置到目标位置的1/10（渐进的思想 如果每一次都移动固定的距离 则可能会移动过头）
                //distance/fabs(distance)是确定移动的方向
                float step = distance / fabs(distance) * fabs(target_pose_ - cur_pose_) * 0.1; // fasb是对浮点数求绝对值
                cur_pose_ += step;
                std::cout << "移动了:" << step << " " << "当前位置:" << cur_pose_ <<std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
           }
           status_ = interfaces::msg::RobotStatus::STATUS_STOP;
           return cur_pose_;
       }

       /*查询机器人当前位置
         @return cur_pose_
       */
        float get_cur_pose() {
            return cur_pose_;
        }

        /*查询机器人当前运动状态
          @return int
            1:interfaces::msg::RobotPose::STATUS_MOVEING
            2:interfaces::msg::RobotPose::STATUS_STOP
        */
        int get_status() {
            return status_;
        }

      private:
        float cur_pose_=0.0;
        float target_pose_ = 0.0;
        int status_ = interfaces::msg::RobotStatus::STATUS_STOP;
};

/*该节点创建一个机器人类 用于模拟一个真实的机器人
1.节点向外提供控制机器人移动的服务，调用该服务可控制机器人移动一段距离，到达所定距离后返回机器人所在位置信息
2.节点对外向话题robot_state发布机器人的位置和（是否在移动）状态
*/

class RobotNode:public rclcpp::Node{
    public:
        RobotNode(std::string name):Node(name) {
            RCLCPP_INFO_STREAM(this->get_logger(), "机器人节点已创建 " << name);
            move_robot_serve_ = this->create_service<interfaces::srv::MoveRobot>(
                "move_robot",
                [this](std::shared_ptr<interfaces::srv::MoveRobot::Request> request,
                       std::shared_ptr<interfaces::srv::MoveRobot::Response> response) {
                    this->handle_move_robot(request, response);
                });

            robot_status_publisher_ = this->create_publisher<interfaces::msg::RobotStatus>(
                "robot_status", 
                10
            );

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                [this]() {
                    this->timer_callback();
                });
        }

    private:
        Robot robot; //实例化机器人
        rclcpp::TimerBase::SharedPtr timer_;//定时器
        rclcpp::Service<interfaces::srv::MoveRobot>::SharedPtr move_robot_serve_;
        rclcpp::Publisher<interfaces::msg::RobotStatus>::SharedPtr robot_status_publisher_;

        /*
        定时发送机器人当前的位置信息
        */
       void timer_callback() {
            interfaces::msg::RobotStatus status_info_;
            status_info_.status = robot.get_status();
            status_info_.pose = robot.get_cur_pose();
            RCLCPP_INFO_STREAM(this->get_logger(), "当前机器人位置： " << status_info_.pose);
            robot_status_publisher_->publish(status_info_);
       }

       /*
       机器人移动服务回调函数
       */
      void handle_move_robot(const std::shared_ptr<interfaces::srv::MoveRobot::Request> request,
                        std::shared_ptr<interfaces::srv::MoveRobot::Response> response) {
          RCLCPP_INFO_STREAM(this->get_logger(), "收到请求移动的距离是： " << request << "当前位置： " << robot.get_cur_pose());
          robot.move_distance(request->distance);
          response->pose = robot.get_cur_pose();
      }
};

int main(int argc,char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNode>("robot");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}