/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * Date       : 2025-04-12
 * File       : subscriber.cpp
 * Description: 
 */ 
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class my_subscriber : public rclcpp::Node
{   
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscribtiber;
    public:
        my_subscriber()
        : Node("Listener")
        {
            subscribtiber = this->create_subscription<std_msgs::msg::String>(
                "topic", 10, std::bind(&my_subscriber::callback_function, this, _1));
        }
    private:
        void callback_function(const std_msgs::msg::String::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I get: '%s'", msg->data.c_str());
        }
    };

    int main(int argc, char * argv[]) {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<my_subscriber>());
        rclcpp::shutdown();
        
        return 0;
    }