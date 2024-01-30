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


namespace hri_head_track_cpp {

const float width_step_ = 0.36;
const float height_step_ = 0.25;
const uint16_t height_ = 480;
const uint16_t width_ = 640;

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
    RCLCPP_WARN(this->get_logger(), "insidesss");


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

    this->obj_pos_sub_ = this->create_subscription<hri_interfaces::action::VideoTracker_FeedbackMessage>(
                           "face_tracker/_action/feedback", 10, std::bind(&HeadTrackActionServer::obj_pos_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "HeadTrackActionServer Initialized");

  }

 private:

  //objects
  rclcpp_action::Server<HeadTrack>::SharedPtr action_server_;
  rclcpp_action::Client<ObjTrack>::SharedPtr client_ptr_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointPositions>::SharedPtr jpos_pub_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointStiffnesses>::SharedPtr jstiff_pub_;

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

  const uint8_t track_max_size_ = 30;
  std::queue<float> x_track_;
  std::queue<float> y_track_;

  nao_lola_command_msgs::msg::JointPositions jpos_cmd_;
  nao_lola_command_msgs::msg::JointStiffnesses jstiff_cmd_;

  bool fileSuccessfullyRead_ = false;

  // ###################################### functions ################################

   void obj_pos_callback(const hri_interfaces::action::VideoTracker_FeedbackMessage & msg) {
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

    printf(" %f \n", msg.feedback.center.x);
    x_track_.push(msg.feedback.center.x);

    y_track_.push(msg.feedback.center.y);

    if (x_track_.size() > track_max_size_) {
      x_track_.pop();
      y_track_.pop();
    }

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
    bool both = false;
    bool tracking = false;

    rclcpp::Rate loop_rate(1);

    while (rclcpp::ok()) {

      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      both = false;
      tracking = false;

      x = x_track_.back();
      y = y_track_.back();

      if ( x != -1 && y != -1 ) { //no face detected

        if (x < 1 / 4 * hri_head_track_cpp::width_ ) {

          if (y < 1 / 4 * hri_head_track_cpp::height_ ) {
            // upper left
            tracking = true;
            both = true;
            jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADYAW);
            jpos_cmd_.positions.push_back(-1 * hri_head_track_cpp::width_step_);
            jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADPITCH);
            jpos_cmd_.positions.push_back(1 * hri_head_track_cpp::height_step_);


          } else if (y > 3 / 4 * height_) {
            // down left
            tracking = true;
            both = true;
            jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADYAW);
            jpos_cmd_.positions.push_back(-1 * hri_head_track_cpp::width_step_);
            jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADPITCH);
            jpos_cmd_.positions.push_back(-1 * hri_head_track_cpp::height_step_);

          } else {
            // left
            tracking = true;
            jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADYAW);
            jpos_cmd_.positions.push_back(-1 * hri_head_track_cpp::width_step_);
          }

        } else if (x > 3 / 4 * hri_head_track_cpp::width_) {
          if (y < 1 / 4 * hri_head_track_cpp::height_ ) {
            // upper right
            tracking = true;
            both = true;
            jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADYAW);
            jpos_cmd_.positions.push_back(1 * hri_head_track_cpp::width_step_);
            jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADPITCH);
            jpos_cmd_.positions.push_back(1 * hri_head_track_cpp::height_step_);

          } else if (y > 3 / 4 * hri_head_track_cpp::height_) {
            //down right
            tracking = true;
            both = true;
            jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADYAW);
            jpos_cmd_.positions.push_back(1 * hri_head_track_cpp::width_step_);
            jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADPITCH);
            jpos_cmd_.positions.push_back(-1 * hri_head_track_cpp::height_step_);

          } else {
            // right
            tracking = true;
            jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADYAW);
            jpos_cmd_.positions.push_back(1 * hri_head_track_cpp::width_step_);
          }
        }

        if ( !both && y < 1 / 4 * hri_head_track_cpp::height_ ) {
          //up
          tracking = true;
          jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADPITCH);
          jpos_cmd_.positions.push_back(1 * hri_head_track_cpp::height_step_);

        } else if ( !both && y > 3 / 4 * hri_head_track_cpp::height_ ) {
          //down
          tracking = true;
          jpos_cmd_.indexes.push_back(joint_indexes_msg_.HEADPITCH);
          jpos_cmd_.positions.push_back(-1 * hri_head_track_cpp::height_step_);

        }

        // send command
        if (tracking) {
          jpos_pub_->publish(jpos_cmd_);
          RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing on effectors/joint_positions");
          jpos_cmd_.indexes.clear();
          jpos_cmd_.positions.clear();
        }

      }// object position available


      loop_rate.sleep();

    }
  }

};  // class HeadTrackActionServer

} // namespace hri_head_track_cpp

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hri_head_track_cpp::HeadTrackActionServer)