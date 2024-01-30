#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>     // std::string, std::stof
#include <iostream>   // std::cout

#include "hri_interfaces/action/joints_play.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"

namespace hri_action_cpp {

class JointsPlayActionServer : public rclcpp::Node {
 public:
  using JointsPlay = hri_interfaces::action::JointsPlay;
  using GoalHandleJointsPlay = rclcpp_action::ServerGoalHandle<JointsPlay>;

  //CUSTOM_ACTION_CPP_PUBLIC
  explicit JointsPlayActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("joints_play_action_server_node", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<JointsPlay>(
                             this,
                             "joints_play",
                             std::bind(&JointsPlayActionServer::handle_goal, this, _1, _2),
                             std::bind(&JointsPlayActionServer::handle_cancel, this, _1),
                             std::bind(&JointsPlayActionServer::handle_accepted, this, _1));

    this->client_ptr_ = rclcpp_action::create_client<VideoTracker>(
                          this,
                          "video_tracker");
    
    this->jpos_pub_ = this->create_publisher<nao_lola_command_msgs::msg::JointPositions>(
                        "effectors/joint_positions", 10);
    
    this->jstiff_pub_ = this->create_publisher<nao_lola_command_msgs::msg::JointStiffnesses>(
                          "effectors/joint_stiffnesses", 10);
    
    RCLCPP_INFO(this->get_logger(), "JointsPlayActionServer Initialized");

  }

 private:

  rclcpp_action::Server<JointsPlay>::SharedPtr action_server_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointPositions>::SharedPtr jpos_pub_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointStiffnesses>::SharedPtr jstiff_pub_;

  std::ifstream ifs_;
  std::vector<float> recorded_joints_;
  nao_lola_command_msgs::msg::JointIndexes joint_indexes_msg_;
  uint8_t rec_joint_indexes_ [10] =  {  joint_indexes_msg_.LSHOULDERPITCH,
                                        joint_indexes_msg_.LSHOULDERROLL,
                                        joint_indexes_msg_.LELBOWYAW,
                                        joint_indexes_msg_.LELBOWROLL,
                                        joint_indexes_msg_.LWRISTYAW,
                                        joint_indexes_msg_.RSHOULDERPITCH,
                                        joint_indexes_msg_.RSHOULDERROLL,
                                        joint_indexes_msg_.RELBOWYAW,
                                        joint_indexes_msg_.RELBOWROLL,
                                        joint_indexes_msg_.RWRISTYAW
                                     };

  uint8_t num_rec_joints_ = sizeof(rec_joint_indexes_) / sizeof(rec_joint_indexes_[0]);
  nao_lola_command_msgs::msg::JointPositions jpos_cmd_;
  nao_lola_command_msgs::msg::JointStiffnesses jstiff_cmd_;

  bool fileSuccessfullyRead_ = false;


  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const JointsPlay::Goal> goal) {

    RCLCPP_INFO( this->get_logger(), ("Received goal request with file path: " + goal->path).c_str() );
    (void)uuid;

    
    ifs_.open( goal->path, std::ifstream::in);
    if (ifs_.is_open()) {
      //RCLCPP_WARN( this->get_logger(), ("File succesfully loaded from " + goal->path).c_str() );
      fileSuccessfullyRead_ = true;

      recorded_joints_.clear();
      float joint_value;
      std::string line;
      while (!ifs_.eof()) {
        std::getline(ifs_, line,'\n');
        joint_value = std::stof(line);
        recorded_joints_.emplace_back( joint_value );
      }
      ifs_.close();
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      RCLCPP_ERROR(this->get_logger(), ("Couldn't open file " + goal->path).c_str() );
      fileSuccessfullyRead_ = false;
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleJointsPlay> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleJointsPlay> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&JointsPlayActionServer::execute, this, _1), goal_handle} .detach();
  }

  void execute(const std::shared_ptr<GoalHandleJointsPlay> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<JointsPlay::Feedback>();
    auto percentage = feedback->percentage;
    auto result = std::make_shared<JointsPlay::Result>();

    for (auto i : rec_joint_indexes_) {
      jstiff_cmd_.indexes.push_back(i);
      jstiff_cmd_.stiffnesses.push_back(1.0);
    }
    jstiff_pub_->publish(jstiff_cmd_);
    jstiff_cmd_.indexes.clear();
    jstiff_cmd_.stiffnesses.clear();
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing 1.0 on effectors/joint_stiffnesses");

    // We create a Rate object of 82Hz
    rclcpp::Rate loop_rate(82);
    for (uint64_t i = 0; i < recorded_joints_.size() && rclcpp::ok(); i += num_rec_joints_) {

      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      for (uint8_t j = 0; j < num_rec_joints_; j++ ) {
        jpos_cmd_.indexes.push_back(rec_joint_indexes_[j]);
        jpos_cmd_.positions.push_back(recorded_joints_[i + j]);
      }
      jpos_pub_->publish(jpos_cmd_);
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing on effectors/joint_positions");
      jpos_cmd_.indexes.clear();
      jpos_cmd_.positions.clear();
      
      loop_rate.sleep();
      //rclcpp::sleep_for(12ms); // 80hz nao_lola update rate
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

};  // class JointsPlayActionServer

}  // namespace hri_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(hri_action_cpp::JointsPlayActionServer)