/*
 * Author     : Youssef Taha Moawad <Eng.Yosef.T@gmail.com>
 * Date       : 2025-04-12
 * File       : publisher.cpp
 * Description: 
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class my_publisher : public rclcpp::Node
{
    private: 
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    public:
        my_publisher()
        : Node("my_publisher"){
            publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
            timer = this->create_wall_timer(500ms, std::bind(&my_publisher::callback_function, this));
        }
    private:
        void callback_function()
        {
            auto message = std_msgs::msg::String();
            message.data = "hello";
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher->publish(message);
        }
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<my_publisher>());
    rclcpp::shutdown();
    
    return 0;
}