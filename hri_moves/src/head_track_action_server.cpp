// Copyright 2024 Antonio Bono
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

#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <iostream>

#include <chrono>
#include <string>     // std::string, std::stof
#include <iostream>   // std::cout
#include <queue>      // std::queue

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hri_interfaces/action/video_tracker.hpp"
//#include "hri_interfaces/action/videoTracker_feedback_message.hpp" //?

#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"
#include "nao_lola_sensor_msgs/msg/joint_indexes.hpp"
#include "nao_lola_sensor_msgs/msg/joint_positions.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace hri_head_track_cpp {

const float width_step_ = 0.36;
const float height_step_ = 0.25;
const float height_ = 480; // y 
const float width_ = 640;  // x

class HeadTrackActionServer : public rclcpp::Node {
 public:
  using HeadTrack = hri_interfaces::action::VideoTracker;
  using GoalHandleHeadTrack = rclcpp_action::ServerGoalHandle<HeadTrack>;
  using ObjTrack = hri_interfaces::action::VideoTracker;
  using GoalHandleObjTrack = rclcpp_action::ClientGoalHandle<ObjTrack>;

  //CUSTOM_ACTION_CPP_PUBLIC
  explicit HeadTrackActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("head_track_action_server_node", options) {

    using namespace std::placeholders;

    this->client_ptr_ = rclcpp_action::create_client<hri_interfaces::action::VideoTracker>(
                          this,
                          "face_tracker");

    this->timer_ = this->create_wall_timer(
                     std::chrono::milliseconds(500),
                     std::bind(&HeadTrackActionServer::send_goal, this));

    this->jpos_sub_ = this->create_subscription<nao_lola_sensor_msgs::msg::JointPositions>(
                        "sensors/joint_positions", 10, std::bind(&HeadTrackActionServer::jpos_callback, this, _1));


    this->jpos_pub_ = this->create_publisher<nao_lola_command_msgs::msg::JointPositions>(
                        "effectors/joint_positions", 10);

    this->jstiff_pub_ = this->create_publisher<nao_lola_command_msgs::msg::JointStiffnesses>(
                          "effectors/joint_stiffnesses", 10);


    this->action_server_ = rclcpp_action::create_server<HeadTrack>(
                             this,
                             "head_track",
                             std::bind(&HeadTrackActionServer::handle_goal, this, _1, _2),
                             std::bind(&HeadTrackActionServer::handle_cancel, this, _1),
                             std::bind(&HeadTrackActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "HeadTrackActionServer Initialized");

    //this->play_cmd_sub_ = this->create_subscription<hri_interfaces::msg::Pos2D>(
    //                "face_pos_topic", 10, std::bind(&HeadTrackActionServer::face_pos_callback, this, _1));

    //this->obj_pos_sub_ = this->create_subscription<hri_interfaces::action::VideoTracker_FeedbackMessage>(
    //                       "face_tracker/_action/feedback", 10, std::bind(&HeadTrackActionServer::obj_pos_callback, this, _1));

  }

 private:

  //objects
  rclcpp_action::Server<HeadTrack>::SharedPtr action_server_;
  rclcpp_action::Client<ObjTrack>::SharedPtr client_ptr_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointPositions>::SharedPtr jpos_pub_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointStiffnesses>::SharedPtr jstiff_pub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::JointPositions>::SharedPtr jpos_sub_;

  rclcpp::Subscription<hri_interfaces::action::VideoTracker_FeedbackMessage>::SharedPtr obj_pos_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  //std::ifstream ifs_;
  //std::vector<float> recorded_joints_;


  // variables and messages
  nao_lola_command_msgs::msg::JointIndexes joint_indexes_msg_;
  uint8_t head_joint_indexes_ [2] =  {  joint_indexes_msg_.HEADYAW,
                                        joint_indexes_msg_.HEADPITCH,
                                     };

  uint8_t num_rec_joints_ = sizeof(head_joint_indexes_) / sizeof(head_joint_indexes_[0]);

  const uint8_t track_max_size_ = 2;
  std::queue<float> x_track_;
  std::queue<float> y_track_;
  float last_yaw_;
  float last_pitch_;
  const double seconds_to_head_reset=6.0; //seconds

  nao_lola_command_msgs::msg::JointPositions jpos_cmd_;
  nao_lola_command_msgs::msg::JointStiffnesses jstiff_cmd_;

  bool fileSuccessfullyRead_ = false;

  // ###################################### functions ################################

  /* void obj_pos_callback(const hri_interfaces::action::VideoTracker_FeedbackMessage & msg) {
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

    printf(" %f \n", msg.feedback.center.x);
    x_track_.push(msg.feedback.center.x);

    y_track_.push(msg.feedback.center.y);

    if (x_track_.size() > track_max_size_) {
      x_track_.pop();
      y_track_.pop();
    }

  } */

  void jpos_callback(const nao_lola_sensor_msgs::msg::JointPositions & joints) {

    last_yaw_ = joints.positions[joint_indexes_msg_.HEADYAW];
    last_pitch_ = joints.positions[joint_indexes_msg_.HEADPITCH];

    RCLCPP_DEBUG_STREAM(this->get_logger(), "New joint positions saved");
  }


// ###################################### client ################################

  void send_goal()  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = ObjTrack::Goal();

    goal_msg.camera_id = " ";

    auto send_goal_options = rclcpp_action::Client<ObjTrack>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&HeadTrackActionServer::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
      std::bind(&HeadTrackActionServer::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&HeadTrackActionServer::result_callback, this, _1);

    RCLCPP_INFO(this->get_logger(), "Sending obj tracking goal " );

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }


  void goal_response_callback(const GoalHandleObjTrack::SharedPtr & goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by object tracking server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by object tracking server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleObjTrack::SharedPtr,
    const std::shared_ptr<const ObjTrack::Feedback> feedback) {

    x_track_.push(feedback->center.x); //from 0 to width_

    y_track_.push(feedback->center.y); //from 0 to height_

    if (x_track_.size() > track_max_size_) {
      x_track_.pop();
      y_track_.pop();
    }

  }

  void result_callback(const GoalHandleObjTrack::WrappedResult & result) {
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


// ###################################### server ################################


  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const HeadTrack::Goal> goal) {

    RCLCPP_INFO( this->get_logger(), "Received goal request" );
    (void)uuid;

    if (true) {

      for (auto i : head_joint_indexes_) {
        jstiff_cmd_.indexes.push_back(i);
        jstiff_cmd_.stiffnesses.push_back(1.0);
      }
      jstiff_pub_->publish(jstiff_cmd_);
      jstiff_cmd_.indexes.clear();
      jstiff_cmd_.stiffnesses.clear();
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing 1.0 on effectors/joint_stiffnesses");

      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    } else {

      //RCLCPP_ERROR(this->get_logger(), ("Couldn't open file " + goal->path).c_str() );
      //fileSuccessfullyRead_ = false;
      return rclcpp_action::GoalResponse::REJECT;
    }

  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleHeadTrack> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleHeadTrack> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&HeadTrackActionServer::execute, this, _1), goal_handle} .detach();
  }

  void execute(const std::shared_ptr<GoalHandleHeadTrack> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<HeadTrack::Feedback>();
    //auto percentage = feedback->center;
    auto result = std::make_shared<HeadTrack::Result>();

    float x;
    float y;
    //float cur_pitch = 0.0;
    //float cur_yaw = 0.0;
    bool both = false;
    bool tracking = false;
    bool right_available, left_available;
    bool up_available, down_available;
    bool first_miss_face = true, reset_head=false;
    rclcpp::Time noFaceTime;


    rclcpp::Rate loop_rate(0.5); //Hz


    while (rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "inside while");
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      both = false;
      tracking = false;

      right_available = last_yaw_ >= -3 * hri_head_track_cpp::width_step_;
      left_available =  last_yaw_ <= 3 * hri_head_track_cpp::width_step_;
      up_available =    last_pitch_ >= -2 * hri_head_track_cpp::height_step_;
      down_available =  last_pitch_ <= 2 * hri_head_track_cpp::height_step_;

      x = x_track_.back();
      y = y_track_.back();

      RCLCPP_INFO(this->get_logger(), ("Received point (x,y) : (" + std::to_string(x) + ", " + std::to_string(y) + ")" ).c_str() );

      if ( x != -1 && y != -1) { // face detected
        RCLCPP_INFO(this->get_logger(), "face detected");

        if (x < 0.25 * hri_head_track_cpp::width_ ) {

          if (y < 0.25 * hri_head_track_cpp::height_ ) {
            // upper left
            RCLCPP_INFO(this->get_logger(), "upper left");
            both = true;

            if (left_available) {
              tracking = true;
              jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADYAW);
              jpos_cmd_.positions.push_back(last_yaw_ + 1 * hri_head_track_cpp::width_step_);
            }
            if (up_available) {
              tracking = true;
              jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADPITCH);
              jpos_cmd_.positions.push_back(last_pitch_ - 1 * hri_head_track_cpp::height_step_);
            }

          } else if (y > 0.75 * hri_head_track_cpp::height_) {
            // down left
            RCLCPP_INFO(this->get_logger(), "down left");
            both = true;

            if (left_available <= 2 * hri_head_track_cpp::width_step_) {
              tracking = true;
              jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADYAW);
              jpos_cmd_.positions.push_back(last_yaw_ + 1 * hri_head_track_cpp::width_step_);
            }

            if (down_available) {
              tracking = true;
              jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADPITCH);
              jpos_cmd_.positions.push_back(last_pitch_ + 1 * hri_head_track_cpp::height_step_);
            }

          } else {
            // left
            RCLCPP_INFO(this->get_logger(), "left");
            if (left_available) {
              tracking = true;
              jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADYAW);
              jpos_cmd_.positions.push_back(last_yaw_ + 1 * hri_head_track_cpp::width_step_);
            }
          }

        } else if (x > 0.75 * hri_head_track_cpp::width_) {
          if (y < 0.25 * hri_head_track_cpp::height_ ) {
            // upper right
            RCLCPP_INFO(this->get_logger(), "upper right");
            both = true;
            if (right_available) {
              tracking = true;
              jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADYAW);
              jpos_cmd_.positions.push_back(last_yaw_ - 1 * hri_head_track_cpp::width_step_);
            }
            if (up_available) {
              tracking = true;
              jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADPITCH);
              jpos_cmd_.positions.push_back(last_pitch_ - 1 * hri_head_track_cpp::height_step_);
            }

          } else if (y > 0.75 * hri_head_track_cpp::height_) {
            //down right
            RCLCPP_INFO(this->get_logger(), "down right");

            both = true;

            if (right_available) {
              tracking = true;
              jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADYAW);
              jpos_cmd_.positions.push_back(last_yaw_ - 1 * hri_head_track_cpp::width_step_);
            }
            if (down_available) {
              tracking = true;
              jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADPITCH);
              jpos_cmd_.positions.push_back(last_pitch_ + 1 * hri_head_track_cpp::height_step_);
            }

          } else {
            // right
            RCLCPP_INFO(this->get_logger(), "right");
            if (right_available) {
              tracking = true;
              jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADYAW);
              jpos_cmd_.positions.push_back(last_yaw_ - 1 * hri_head_track_cpp::width_step_);
            }
          }
        } else if ( !both && y < 0.25 * hri_head_track_cpp::height_ ) {
          //up
          RCLCPP_INFO(this->get_logger(), "up");

          if (up_available) {
            tracking = true;
            jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADPITCH);
            jpos_cmd_.positions.push_back(last_pitch_ - 1 * hri_head_track_cpp::height_step_);
          }

        } else if ( !both && y > 0.75 * hri_head_track_cpp::height_ ) {
          //down
          RCLCPP_INFO(this->get_logger(), "down");

          if (down_available) {
            tracking = true;
            jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADPITCH);
            jpos_cmd_.positions.push_back(last_pitch_ + 1 * hri_head_track_cpp::height_step_);
          }
        } else {
          //center of the frame
          RCLCPP_INFO(this->get_logger(), "center");

        }
        // send command
        if (tracking) {
          jpos_pub_->publish(jpos_cmd_);
          RCLCPP_INFO(this->get_logger(), "Publishing on effectors/joint_positions");
          jpos_cmd_.indexes.clear();
          jpos_cmd_.positions.clear();
          first_miss_face = true;
          reset_head=false;
        }
        // object position available
      } else {
        RCLCPP_INFO(this->get_logger(), "No face detected");
        if (first_miss_face) {
          first_miss_face = false;

          noFaceTime = rclcpp::Clock{RCL_ROS_TIME}.now();
          RCLCPP_INFO_STREAM(this->get_logger(), "first no face time: "<< noFaceTime.seconds());
        } else if (rclcpp::Clock{RCL_ROS_TIME}.now().seconds() - noFaceTime.seconds() > seconds_to_head_reset 
                      && !reset_head) {
          jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADPITCH);
          jpos_cmd_.positions.push_back(0.0);
          jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADYAW);
          jpos_cmd_.positions.push_back(0.0);
          jpos_pub_->publish(jpos_cmd_);
          reset_head=true;
        }
      }

      loop_rate.sleep();

    }
  }

};  // class HeadTrackActionServer

} // namespace hri_head_track_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(hri_head_track_cpp::HeadTrackActionServer)