#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "turtlesim/srv/spawn.hpp"

#include "turtle_multi_target_interfaces/msg/target_name.hpp"
#include "turtle_multi_target_interfaces/msg/current_target.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class TurtleController : public rclcpp::Node
{
public:
  TurtleController()
  : Node("turtle_controller"),
    turtle_spawning_service_ready_(false),
    turtle_spawned_(false)
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a client to spawn a turtle
    spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");

    // Create turtle2 velocity publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "turtle2/cmd_vel", 1);

    current_target_pub_ = this->create_publisher<turtle_multi_target_interfaces::msg::CurrentTarget>(
      "/current_target", 1);

    target_name_sub_ = this->create_subscription<turtle_multi_target_interfaces::msg::TargetName>(
      "/target_name", 10, std::bind(&TurtleController::target_name_callback, this, _1));

    // Call on_timer function every second
    timer_ = this->create_wall_timer(1s, std::bind(&TurtleController::on_timer, this));
  }

private:
  void target_name_callback(const turtle_multi_target_interfaces::msg::TargetName & target_name_msg){
    if (target_frame_ != target_name_msg.target_name) {
      target_frame_ = target_name_msg.target_name;
      RCLCPP_INFO(this->get_logger(), "Changed target. Now target is '%s'", target_frame_.c_str());
    }
  }

  void on_timer(){
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = "turtle2";

    if (turtle_spawning_service_ready_) {
      if (turtle_spawned_) {
        geometry_msgs::msg::TransformStamped t;

        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
        try {
          t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }

        geometry_msgs::msg::Twist twist_msg;

        static const double scaleRotationRate = 1.0;
        twist_msg.angular.z = scaleRotationRate * atan2(
          t.transform.translation.y,
          t.transform.translation.x);

        static const double scaleForwardSpeed = 0.5;
        double distance_to_target = sqrt(pow(t.transform.translation.x, 2) + pow(t.transform.translation.y, 2));
        twist_msg.linear.x = scaleForwardSpeed * distance_to_target;
        
        turtle_multi_target_interfaces::msg::CurrentTarget current_target_msg;
        current_target_msg.distance_to_target = distance_to_target;
        current_target_msg.target_x = t.transform.translation.x;
        current_target_msg.target_y = t.transform.translation.y;
        current_target_msg.target_name = target_frame_;
        
        current_target_pub_->publish(current_target_msg);
        cmd_vel_pub_->publish(twist_msg);
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully spawned");
        turtle_spawned_ = true;
      }
    } else {
      // Check if the service is ready
      if (spawner_->service_is_ready()) {
        // Initialize request with turtle name and coordinates
        // Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = 4.0;
        request->y = 2.0;
        request->theta = 0.0;
        request->name = "turtle2";

        // Call request
        using ServiceResponseFuture =
          rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            auto result = future.get();
            if (strcmp(result->name.c_str(), "turtle2") == 0) {
              turtle_spawning_service_ready_ = true;
            } else {
              RCLCPP_ERROR(this->get_logger(), "Service callback result mismatch");
            }
          };
        auto result = spawner_->async_send_request(request, response_received_callback);
      } else {
        RCLCPP_INFO(this->get_logger(), "Service is not ready");
      }
    }
  }

  // Boolean values to store the information
  // if the service for spawning turtle is available
  bool turtle_spawning_service_ready_;
  // if the turtle was successfully spawned
  bool turtle_spawned_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_{nullptr};
  rclcpp::Publisher<turtle_multi_target_interfaces::msg::CurrentTarget>::SharedPtr current_target_pub_;
  rclcpp::Subscription<turtle_multi_target_interfaces::msg::TargetName>::SharedPtr target_name_sub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleController>());
  rclcpp::shutdown();
  return 0;
}