#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "hri_interfaces/action/joints_play.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace fs = boost::filesystem;


namespace hri_action_cpp {

class JointsPlayActionClient : public rclcpp::Node {
 public:
  using JointsPlay = hri_interfaces::action::JointsPlay;
  using GoalHandleJointsPlay = rclcpp_action::ClientGoalHandle<JointsPlay>;

  explicit JointsPlayActionClient(const rclcpp::NodeOptions & options)
    : Node("joints_play_action_client", options) {
    this->client_ptr_ = rclcpp_action::create_client<JointsPlay>(
                          this,
                          "joints_play");

    this->timer_ = this->create_wall_timer(
                     std::chrono::milliseconds(500),
                     std::bind(&JointsPlayActionClient::send_goal, this));

    this->declare_parameter<std::string>("file", getDefaultFullFilePath());
  }

  void send_goal()  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = JointsPlay::Goal();

    std::string filePath;
    this->get_parameter("file", filePath);

    goal_msg.path = filePath;

    RCLCPP_INFO(this->get_logger(), ("Sending goal: " + filePath).c_str() );

    auto send_goal_options = rclcpp_action::Client<JointsPlay>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&JointsPlayActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&JointsPlayActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&JointsPlayActionClient::result_callback, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }


 private:
  rclcpp_action::Client<JointsPlay>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string getDefaultFullFilePath() {
    
    /*
    RCLCPP_INFO(this->get_logger(),"getDefaultFullFilePath ");
    std::string file = "moves/hello.txt";
    std::string package_share_directory = ament_index_cpp::get_package_share_directory(
                                            "hri_moves");
    // e.g. /home/nao/rolling_ws/install/hri_moves/share/hri_moves

    RCLCPP_INFO(this->get_logger(),("package_share_directory: "+package_share_directory).c_str());
    fs::path dir_path(package_share_directory);
    RCLCPP_INFO(this->get_logger(),("dir path "+dir_path.string()).c_str() );
    fs::path file_path(file);
    RCLCPP_INFO(this->get_logger(),("file path"+file_path.string()).c_str());
    fs::path full_path = dir_path / file_path;
    RCLCPP_INFO(this->get_logger(),"getDefaultFullFilePath 5.... ");
    RCLCPP_INFO(this->get_logger(), ("File path " + full_path.string()).c_str());
    return full_path.string();
    */

    return "/home/nao/rolling_ws/src/hri/hri_moves/hello.txt";
    //return "hello.txt";  //WORKING
  }


  void goal_response_callback(const GoalHandleJointsPlay::SharedPtr & goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleJointsPlay::SharedPtr,
    const std::shared_ptr<const JointsPlay::Feedback> feedback) {

    //TODO

  }

  void result_callback(const GoalHandleJointsPlay::WrappedResult & result) {
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
      RCLCPP_INFO(this->get_logger(), "Joints posisitions regulary played.");

    rclcpp::shutdown();
  }

};  // class JointsPlayActionClient

}  // namespace hri_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(hri_action_cpp::JointsPlayActionClient)