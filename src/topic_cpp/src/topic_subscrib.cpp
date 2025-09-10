//create a subscrib
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TopicSubscrib : public rclcpp::Node {
    public:
        TopicSubscrib(std::string name) : rclcpp::Node(name) {

        }
    private:
        
};