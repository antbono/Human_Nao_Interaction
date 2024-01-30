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
#include "naosoccer_pos_action_interfaces/action/action.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

namespace fs = boost::filesystem;

namespace hri_naosoccer_action_client {

class NaosoccerActionClient : public rclcpp::Node {
 public:

  using PosAction = naosoccer_pos_action_interfaces::action::Action;
  using GoalHandlePosAction = rclcpp_action::ClientGoalHandle<PosAction>;

  explicit NaosoccerActionClient(const rclcpp::NodeOptions & options)
    : Node("naosoccer_action_client", options) {
    this->client_ptr_ = rclcpp_action::create_client<PosAction>(
                          this,
                          "naosoccer_pos_action");

    /*this->timer_ = this->create_wall_timer(
                     std::chrono::milliseconds(500),
                     std::bind(&NaosoccerActionClient::send_goal, this));
    */

    this->sub_action_req_ = this->create_subscription<std_msgs::msg::String>(
            "action_req_",
            10,
            std::bind(&NaosoccerActionClient::action_req_callback, this, _1));  

    RCLCPP_INFO(this->get_logger(), "NaosoccerActionClient initialized");

  }

  void send_goal( std::string & action_name)  {
    using namespace std::placeholders;

    //this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = PosAction::Goal();
    goal_msg.action_name = action_name;

    auto send_goal_options = rclcpp_action::Client<PosAction>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&NaosoccerActionClient::goal_response_callback, this, _1);
    
    //send_goal_options.feedback_callback =
    //  std::bind(&NaosoccerActionClient::feedback_callback, this, _1, _2);
    
    send_goal_options.result_callback =
      std::bind(&NaosoccerActionClient::result_callback, this, _1);

    RCLCPP_INFO(this->get_logger(), "Sending goal: "  );

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }


 private:

  rclcpp_action::Client<PosAction>::SharedPtr client_ptr_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_action_req_;

  void action_req_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    
    std::string action_name = msg->data;
    this->send_goal(action_name);
  }

  void goal_response_callback(const GoalHandlePosAction::SharedPtr & goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandlePosAction::SharedPtr,
    const std::shared_ptr<const PosAction::Feedback> feedback) {

    //TODO

  }

  void result_callback(const GoalHandlePosAction::WrappedResult & result) {
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

    //if (result.result->success)
      RCLCPP_INFO(this->get_logger(), "Joints posisitions regulary played.");

    //rclcpp::shutdown();
  }

};  // class HeadTrackActionClient

}  // namespace hri_head_track_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(hri_naosoccer_action_client::NaosoccerActionClient)