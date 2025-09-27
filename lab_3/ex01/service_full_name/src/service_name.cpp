#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "service_full_name_interfaces/srv/summ_full_name.hpp"


void concat(const std::shared_ptr<service_full_name_interfaces::srv::SummFullName::Request> request,
            std::shared_ptr<service_full_name_interfaces::srv::SummFullName::Response> response){
  response->full_name = request->last_name + " " + request->name + " " + request->first_name;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nlast_name: %s\n" "name: %s\n" "first_name: %s",
              request->last_name.c_str(), request->name.c_str(), request->first_name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->full_name.c_str());
}


int main(int argc, char ** argv){
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_name");

  rclcpp::Service<service_full_name_interfaces::srv::SummFullName>::SharedPtr service =
    node->create_service<service_full_name_interfaces::srv::SummFullName>("summ_full_name", &concat);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to concat full name.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
