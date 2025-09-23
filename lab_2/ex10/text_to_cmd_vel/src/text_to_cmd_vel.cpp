// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;


class TextToCmdVel : public rclcpp::Node
{
public:
    TextToCmdVel()
    : Node("text_to_cmd_vel")
    {   
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 1);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "cmd_text", 10, std::bind(&TextToCmdVel::exec_command, this, _1));
        RCLCPP_INFO(this->get_logger(), "Converter started\nAvailable commands: move_{forward, backward}, turn_{left, right}");
    }

private:
    void exec_command(const std_msgs::msg::String & command) const
    {
        double linear = 0, angular = 0;
        if (command.data == "turn_right") {
            angular = -1.0;
        }
        else if (command.data == "turn_left") {
            angular = 1.0;
        }
        else if (command.data == "move_forward") {
            linear = 1.0;
        }
        else if (command.data == "move_backward") {
            linear = -1.0;
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Unrecognized command: '%s'", command.data.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Exec command: '%s'", command.data.c_str());
        geometry_msgs::msg::Twist twist;
        twist.angular.z = angular;
        twist.linear.x = linear;
        publisher_->publish(twist);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TextToCmdVel>());
    rclcpp::shutdown();
    return 0;
}
