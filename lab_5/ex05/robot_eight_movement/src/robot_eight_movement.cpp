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

class RobotEightMovement : public rclcpp::Node{
public:
    RobotEightMovement() : Node("robot_eight_movement"){
        robot_prefix_ = this->declare_parameter<std::string>("robot_prefix", "crab_bot");

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            robot_prefix_ + "/cmd_vel", 1);
        cmd_vel_timer_ = this->create_wall_timer(
            1s, std::bind(&RobotEightMovement::cmd_vel_timer_callback, this));
    }
private:
    void make_one_publish(geometry_msgs::msg::Twist & t, double angular_vel, double linear_vel){
        t.angular.z = angular_vel;
        t.linear.x = linear_vel;
        cmd_vel_publisher_->publish(t);
    }

    void cmd_vel_timer_callback(){
        if (reached_begin_point == false) {
            geometry_msgs::msg::Twist t;

            make_one_publish(t, M_PI / 4, 0);
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();

            make_one_publish(t, 0, 0.25);
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();

            make_one_publish(t, - M_PI / 4, 0);
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();

            make_one_publish(t, 0, 0);
            one_sec_rate_.sleep();

            reached_begin_point = true;
        } else {
            geometry_msgs::msg::Twist t;

            make_one_publish(t, - M_PI / 4, 0.5);
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();

            make_one_publish(t, M_PI / 4, 0.5);
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();

            make_one_publish(t, - M_PI / 4, 0.5);
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();
            one_sec_rate_.sleep();

            make_one_publish(t, 0, 0);
            one_sec_rate_.sleep();
        }
    }

    bool reached_begin_point = false;
    rclcpp::Rate one_sec_rate_{1};
    std::string robot_prefix_;

    rclcpp::TimerBase::SharedPtr cmd_vel_timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotEightMovement>());
    rclcpp::shutdown();
}