#include <cstdio>
#include <iostream>
#include <memory>
#include <math.h>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"


using std::placeholders::_1;


double min_angle(double alpha, double beta){
    // double theta = std::fmod(beta - alpha + 3.14, 2 * 3.14)
    return std::fmod(beta - alpha + 3.14, 2 * 3.14) - 3.14;
}


double vec_len(double x, double y){
    return sqrt(pow(x, 2) + pow(y, 2));
}


double angle_between_vectors(double x1, double y1, double x2, double y2){
    return acos((x1*x2 + y1*y2) / (vec_len(x1, y1) * vec_len(x2, y2)));
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
            goal_vec_x = round_n(goal_x - p.x, 3);
            goal_vec_y = round_n(goal_y - p.y, 3);
            goal_dist = vec_len(goal_vec_x, goal_vec_y);

            if (goal_dist > 0.1) {
                turtle_vec_x = cos(p.theta);
                turtle_vec_y = sin(p.theta);
                alpha = angle_between_vectors(goal_vec_x, goal_vec_y, turtle_vec_x, turtle_vec_y);
                direction = turtle_vec_x * goal_vec_y - turtle_vec_y * goal_vec_x;
                if (direction < 0) {
                    alpha = -alpha;
                }

                if (abs(alpha) >= 1e-2) {
                    twist.angular.z = alpha;
                    twist.linear.x = twist.linear.y = 0;
                    publisher_->publish(twist);
                } else {
                    twist.angular.z = 0;
                    twist.linear.x = goal_dist;
                    twist.linear.y = 0;
                }
                publisher_->publish(twist);
            }
            else if (abs(min_angle(p.theta, final_theta)) >= 1e-2) {
                twist.angular.z = min_angle(p.theta, final_theta);
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
    geometry_msgs::msg::Twist twist;
    double goal_x, goal_y, final_theta;
    double goal_vec_x, goal_vec_y, goal_dist;
    double turtle_vec_x, turtle_vec_y;
    double alpha, direction;
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
