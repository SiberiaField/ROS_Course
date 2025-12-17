#include <cstdio>
#include <iostream>
#include <memory>
#include <math.h>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotCircleMovement : public rclcpp::Node{
public:
    RobotCircleMovement() : Node("robot_circle_movement"){
        robot_prefix_ = this->declare_parameter<std::string>("robot_prefix", "crab_bot");
        linear_vel_ = this->declare_parameter<double>("linear_vel", 0.3);
        angular_vel_ = this->declare_parameter<double>("angular_vel", M_PI / 2);

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            robot_prefix_ + "/cmd_vel", 1);
        cmd_vel_timer_ = this->create_wall_timer(
            1s, std::bind(&RobotCircleMovement::cmd_vel_timer_callback, this));
    }
private:
    void cmd_vel_timer_callback(){
        cmd_vel_timer_->cancel();
        geometry_msgs::msg::Twist t;
        t.angular.z = angular_vel_;
        t.linear.x = linear_vel_;
        cmd_vel_publisher_->publish(t);

        rclcpp::Rate shutdown_rate(1);
        shutdown_rate.sleep();
        rclcpp::shutdown();
    }

    double angular_vel_;
    double linear_vel_;
    std::string robot_prefix_;

    rclcpp::TimerBase::SharedPtr cmd_vel_timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotCircleMovement>());
    rclcpp::shutdown();
}