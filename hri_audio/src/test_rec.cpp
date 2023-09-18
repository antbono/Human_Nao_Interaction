#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  //if (argc != 3) {
  //    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
  //    return 1;
  //}

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("record_to_file_client");
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client =
    node->create_client<std_srvs::srv::SetBool>("record_to_file");

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  //request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "start req sent");
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start record success");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service record_to_file to start record");
  }


  request->data = false;

  //request->b = atoll(argv[2])
;
  rclcpp::sleep_for(5s);

  result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stop record success");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service record_to_file to stop record");
  }

  rclcpp::shutdown();
  return 0;
}