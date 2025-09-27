#include "rclcpp/rclcpp.hpp"
#include "service_full_name_interfaces/srv/summ_full_name.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: client_name FIRST_NAME NAME LAST_NAME");
    return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("client_name");
  rclcpp::Client<service_full_name_interfaces::srv::SummFullName>::SharedPtr client =
    node->create_client<service_full_name_interfaces::srv::SummFullName>("summ_full_name");

  auto request = std::make_shared<service_full_name_interfaces::srv::SummFullName::Request>();
  request->last_name = argv[1];
  request->name = argv[2];
  request->first_name = argv[3];

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Full Name: %s", result.get()->full_name.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service summ_full_name");
  }

  rclcpp::shutdown();
  return 0;
}