// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>

#include <math.h>
#include <termios.h>
#include <unistd.h>

#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "turtle_multi_target_interfaces/msg/target_name.hpp"
#include "turtle_multi_target_interfaces/msg/current_target.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


typedef struct turtle_pose_cb_params{
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription;
} turtle_pose_cb_params;


class TargetSwitcher : public rclcpp::Node{
public:
  TargetSwitcher() : Node("target_switcher"){
    radius_ = this->declare_parameter<double>("radius", 1);
    direction_of_rotation_ = this->declare_parameter<int>("direction_of_rotation", 1);
    switch_threshold_ = this->declare_parameter<double>("switch_threshold", 1);
    distance_to_target_ = switch_threshold_ + 1;

    turtles_pose_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    turtle_pose_cb_params turtle_one;
    this->init_turtle_pose_callback(turtle_one, "turtle1", turtles_pose_cb_group_);
    turtle_pose_callbacks_.push_back(turtle_one);

    turtle_pose_cb_params turtle_two;
    this->init_turtle_pose_callback(turtle_two, "turtle2", turtles_pose_cb_group_);
    turtle_pose_callbacks_.push_back(turtle_two);

    turtle_pose_cb_params turtle_three;
    this->init_turtle_pose_callback(turtle_three, "turtle3", turtles_pose_cb_group_);
    turtle_pose_callbacks_.push_back(turtle_three);

    targets_tf_broadcaster_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    targets_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    broadcast_timer_ = this->create_wall_timer(
      100ms, std::bind(&TargetSwitcher::broadcast_timer_callback, this), targets_tf_broadcaster_cb_group_);
    
    target_name_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    target_name_pub_ = this->create_publisher<turtle_multi_target_interfaces::msg::TargetName>(
      "/target_name", 1);
    target_name_timer_ = this->create_wall_timer(
      1s, std::bind(&TargetSwitcher::target_name_timer_callback, this), target_name_cb_group_);

    rclcpp::SubscriptionOptions options;
    options.callback_group = target_name_cb_group_;
    current_target_sub_ = this->create_subscription<turtle_multi_target_interfaces::msg::CurrentTarget>(
      "/current_target", 10, std::bind(&TargetSwitcher::current_target_callback, this, _1), options);

    keyboard_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    keyboard_timer_ = this->create_wall_timer(
      10ms, std::bind(&TargetSwitcher::keyboard_timer_callback, this), keyboard_cb_group_);
  }

  ~TargetSwitcher() {
    restore_terminal();
  }

  void restore_terminal() {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
  }

private:
  void init_turtle_pose_callback(turtle_pose_cb_params & t,
                                 std::string && turtlename, 
                                 rclcpp::CallbackGroup::SharedPtr cb_group){
    t.tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    rclcpp::SubscriptionOptions options;
    options.callback_group = cb_group;

    std::ostringstream stream;
    stream << "/" << turtlename.c_str() << "/pose";
    std::string topic_name = stream.str();
    
    t.subscription = this->create_subscription<turtlesim::msg::Pose>(
      topic_name, 10, 
      [this, turtlename, t](const turtlesim::msg::Pose::SharedPtr msg) {
        this->handle_turtle_pose(turtlename, t.tf_broadcaster, msg);
      },
      options);
  }

  geometry_msgs::msg::TransformStamped first_carrot_tf(rclcpp::Time now){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "turtle1";
    t.child_frame_id = "carrot1";
    
    double theta = (M_PI / 4) * fmod(now.seconds(), 8);
    if (theta < 0) {
      theta += 2 * M_PI;
    }

    t.transform.translation.x = cos(theta) * radius_;
    t.transform.translation.y = sin(theta) * radius_ * direction_of_rotation_ * -1;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    return t;
  }

  geometry_msgs::msg::TransformStamped second_carrot_tf(rclcpp::Time now){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "turtle3";
    t.child_frame_id = "carrot2";

    double theta = (M_PI / 4) * fmod(now.seconds(), 8);
    if (theta < 0) {
      theta += 2 * M_PI;
    }

    t.transform.translation.x = 2 * cos(theta);
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    return t;
  }

  geometry_msgs::msg::TransformStamped static_target_tf(rclcpp::Time now){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "world";
    t.child_frame_id = "static_target";

    t.transform.translation.x = 8.0;
    t.transform.translation.y = 2.0;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    return t;
  }

  void handle_turtle_pose(const std::string& turtlename,
                          std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster, 
                          const std::shared_ptr<turtlesim::msg::Pose> msg){
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = turtlename.c_str();

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster->sendTransform(t);
  }

  void broadcast_timer_callback(){
    rclcpp::Time now = this->get_clock()->now();
    targets_tf_broadcaster_->sendTransform(this->first_carrot_tf(now));
    targets_tf_broadcaster_->sendTransform(this->second_carrot_tf(now));
    targets_tf_broadcaster_->sendTransform(this->static_target_tf(now));
    // RCLCPP_INFO(this->get_logger(), "Broadcast transforms");
  }

  void current_target_callback(const turtle_multi_target_interfaces::msg::CurrentTarget & current_target){
    // RCLCPP_INFO(this->get_logger(), "Get current_target msg");
    distance_to_target_ = current_target.distance_to_target;
  }

  void target_name_timer_callback(){
    if ((distance_to_target_ < switch_threshold_) || n_entered_) {
      target_id_ = (target_id_ + 1) % 3;
      if (n_entered_) {
        std::lock_guard<std::mutex> lock_n_entered(n_entered_mutex_);
        n_entered_ = false;
      }
    }

    turtle_multi_target_interfaces::msg::TargetName target_name_msg;
    switch (target_id_) {
    case 0:
      target_name_msg.target_name = "carrot1";
      break;
    case 1:
      target_name_msg.target_name = "carrot2";
      break;
    case 2:
      target_name_msg.target_name = "static_target";
      break;
    default: break;
    }

    // RCLCPP_INFO(this->get_logger(), "Publish target_name");
    target_name_pub_->publish(target_name_msg);
  }

  void setup_nonblocking_input() {
    tcgetattr(STDIN_FILENO, &original_termios_);
    struct termios new_termios = original_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    keyboard_initialized_ = true;
  }

  void check_keyboard_input() {
    char sym;
    if (read(STDIN_FILENO, &sym, 1) > 0) {
      if (sym == 'n') {
        std::lock_guard<std::mutex> lock(n_entered_mutex_);
        n_entered_ = true;
        RCLCPP_INFO(this->get_logger(), "Target change requested.");
      }
      else if (sym == 'q') {
        RCLCPP_INFO(this->get_logger(), "Got quit request");
        keyboard_timer_->cancel();
        restore_terminal();
        rclcpp::shutdown();
      }
    }
        
    if (!rclcpp::ok()) {
      keyboard_timer_->cancel();
      restore_terminal();
    }
  }

  void keyboard_timer_callback() {
    if (!keyboard_initialized_) {
      setup_nonblocking_input();
      keyboard_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Keyboard input ready. Press 'n' to change target. Press 'q' to quit");
    }
        
    check_keyboard_input();
  }

  rclcpp::CallbackGroup::SharedPtr targets_tf_broadcaster_cb_group_;
  rclcpp::TimerBase::SharedPtr broadcast_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> targets_tf_broadcaster_;

  rclcpp::CallbackGroup::SharedPtr turtles_pose_cb_group_;
  std::vector<turtle_pose_cb_params> turtle_pose_callbacks_;

  rclcpp::Subscription<turtle_multi_target_interfaces::msg::CurrentTarget>::SharedPtr current_target_sub_;

  rclcpp::CallbackGroup::SharedPtr target_name_cb_group_;
  rclcpp::TimerBase::SharedPtr target_name_timer_;
  rclcpp::Publisher<turtle_multi_target_interfaces::msg::TargetName>::SharedPtr target_name_pub_;

  rclcpp::CallbackGroup::SharedPtr keyboard_cb_group_;
  rclcpp::TimerBase::SharedPtr keyboard_timer_;

  double radius_;
  double switch_threshold_;
  int direction_of_rotation_;

  int target_id_ = 0;
  bool n_entered_ = false;
  double distance_to_target_;

  struct termios original_termios_;
  bool keyboard_initialized_ = false;

  std::mutex n_entered_mutex_;
};


int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating target_switcher node");
  auto target_switcher_node = std::make_shared<TargetSwitcher>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating executor");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
  executor.add_node(target_switcher_node);
  RCLCPP_INFO(target_switcher_node->get_logger(), "Starting target_switcher node");
  executor.spin();
  target_switcher_node->restore_terminal();

  RCLCPP_INFO(target_switcher_node->get_logger(), "Shutdown target_switcher node");
  rclcpp::shutdown();
  return 0;
}
