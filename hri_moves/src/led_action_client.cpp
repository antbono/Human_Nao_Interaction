#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "nao_lola_command_msgs/msg/head_leds.hpp"
#include "nao_lola_command_msgs/msg/right_eye_leds.hpp"
#include "nao_lola_command_msgs/msg/left_eye_leds.hpp"
#include "nao_lola_command_msgs/msg/right_ear_leds.hpp"
#include "nao_lola_command_msgs/msg/left_ear_leds.hpp"
#include "nao_lola_command_msgs/msg/chest_led.hpp"
#include "nao_lola_command_msgs/msg/right_foot_led.hpp"
#include "nao_lola_command_msgs/msg/left_foot_led.hpp"

#include "hri_interfaces/action/leds_play.hpp"
#include "hri_interfaces/msg/led_indexes.hpp"
#include "hri_interfaces/msg/led_modes.hpp"
#include "hri_interfaces/action/leds_play.hpp"

#include "std_msgs/msg/color_rgba.hpp"

namespace fs = boost::filesystem;

namespace hri_led_action_client {

class LedsPlayActionClient : public rclcpp::Node {
 public:
  using LedsPlay = hri_interfaces::action::LedsPlay;
  using GoalHandleLedsPlay = rclcpp_action::ClientGoalHandle<LedsPlay>;
  
  using LedIndexes = hri_interfaces::msg::LedIndexes;
  using LedModes = hri_interfaces::msg::LedModes;

  explicit LedsPlayActionClient(const rclcpp::NodeOptions & options)
    : Node("leds_play_action_client", options) {
    this->client_ptr_ = rclcpp_action::create_client<LedsPlay>(
                          this,
                          "leds_play");

    this->timer_ = this->create_wall_timer(
                     std::chrono::milliseconds(500),
                     std::bind(&LedsPlayActionClient::send_goal, this));

    //this->declare_parameter<std::string>("file", getDefaultFullFilePath());

    RCLCPP_INFO(this->get_logger(), "LedsPlayActionClient initialized");

  }

  void send_goal()  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = LedsPlay::Goal();

    goal_msg.leds = {LedIndexes::REYE, LedIndexes::LEYE};
    goal_msg.mode = LedModes::BLINKING;
    std_msgs::msg::ColorRGBA color;
    color.r=0.0; color.g=1.0; color.b=0.0;
    for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEyeLeds::NUM_LEDS; ++i) {
                goal_msg.colors[i] = color;
    }
    goal_msg.frequency = 1.0;
   
   auto send_goal_options = rclcpp_action::Client<LedsPlay>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&LedsPlayActionClient::goal_response_callback, this, _1);
    
    send_goal_options.feedback_callback =
      std::bind(&LedsPlayActionClient::feedback_callback, this, _1, _2);
    
    send_goal_options.result_callback =
      std::bind(&LedsPlayActionClient::result_callback, this, _1);

    RCLCPP_INFO(this->get_logger(), "Sending goal:" );

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }


 private:

  rclcpp_action::Client<LedsPlay>::SharedPtr client_ptr_;

  rclcpp::TimerBase::SharedPtr timer_;


  void goal_response_callback(const GoalHandleLedsPlay::SharedPtr & goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleLedsPlay::SharedPtr,
    const std::shared_ptr<const LedsPlay::Feedback> feedback) {

    //TODO

  }

  void result_callback(const GoalHandleLedsPlay::WrappedResult & result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    if (result.result->success)
      RCLCPP_INFO(this->get_logger(), "Leds regulary played.");

    rclcpp::shutdown();
  }

};  // class LedsPlayActionClient

}  // namespace hri_led_action_client

RCLCPP_COMPONENTS_REGISTER_NODE(hri_led_action_client::LedsPlayActionClient)