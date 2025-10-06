#include <cstdio>
#include <iostream>
#include <memory>
#include <math.h>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"


using std::placeholders::_1;


double min_angle(double a, double b){
    double angle1 = a - b;
    double angle2 = b - a + 2*3.14;
    return (angle1 > angle2) ? angle2 : angle1;
}


double vec_len(double x, double y){
    return sqrt(pow(x, 2) + pow(y, 2));
}


double round_n(double x, double n){
    return round(x * pow(10, n)) / pow(10, n);
}


class MoveToGoal : public rclcpp::Node{
public:
    MoveToGoal(double goal_x, double goal_y, double final_theta) : Node("move_to_goal"){   
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 1);
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&MoveToGoal::reach_the_goal, this, _1));
        this->goal_x = goal_x;
        this->goal_y = goal_y;
        this->final_theta = final_theta;
        RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f, Theta: %f", goal_x, goal_y, final_theta);
    }

private:
    void reach_the_goal(const turtlesim::msg::Pose & p){
        if ((p.linear_velocity == 0) && (p.angular_velocity == 0)){
            double goal_vec_x = round_n(goal_x - p.x, 3);
            double goal_vec_y = round_n(goal_y - p.y, 3);
            double goal_dist = vec_len(goal_vec_x, goal_vec_y);

            geometry_msgs::msg::Twist twist;
            if (goal_dist > 0.1){
                RCLCPP_DEBUG(this->get_logger(), "goal_vec = (%f, %f), with len = %f", goal_vec_x, goal_vec_y, goal_dist);
                if (abs(p.theta) > 1e-3){
                    twist.angular.z = -p.theta;
                    twist.linear.x = twist.linear.y = 0;
                } else {
                    twist.angular.z = 0;
                    twist.linear.x = goal_vec_x;
                    twist.linear.y = goal_vec_y;
                }
                publisher_->publish(twist);
            }
            else if (abs(min_angle(final_theta, p.theta)) > 1e-2){
                RCLCPP_DEBUG(this->get_logger(), "Theta %f", p.theta);
                twist.angular.z = round_n(min_angle(final_theta, p.theta), 3);
                twist.linear.x = twist.linear.y = 0;
                publisher_->publish(twist);
            }
            else{
                RCLCPP_INFO(this->get_logger(), "REACHED THE GOAL");
                rclcpp::shutdown();
            }
        }
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double goal_x, goal_y, final_theta;
};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    if (argc != 4) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: move_to_goal X Y THETA");
        return 1;
    }

    rclcpp::spin(std::make_shared<MoveToGoal>(atof(argv[1]), atof(argv[2]), atof(argv[3])));
    rclcpp::shutdown();
}
