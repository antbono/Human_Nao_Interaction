#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <chrono>
#include <memory>

void wait(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Received wait request");
  rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<uint64_t>(5 * 1e9)));
  response->success = true;
  response->message = "none";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"wait request done");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("wait_server");

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service =
    node->create_service<std_srvs::srv::SetBool>("gstt_service", &wait);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "wait_server initialized");

  rclcpp::spin(node);
  rclcpp::shutdown();
}