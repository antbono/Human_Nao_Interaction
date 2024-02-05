#include <functional>
#include <memory>
#include <thread>
//#include <vector>
#include <array>
//#include <chrono>
#include <string>     // std::string, std::stof
#include <cstring>
#include <iostream>   // std::cout

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


#include "hri_interfaces/msg/led_indexes.hpp"
#include "hri_interfaces/msg/led_modes.hpp"
#include "hri_interfaces/action/leds_play.hpp"

#include "std_msgs/msg/color_rgba.hpp"


namespace hri_led_action_server {



class LedsPlayActionServer : public rclcpp::Node {
  public:
    using LedsPlay = hri_interfaces::action::LedsPlay;
    using GoalHandleLedsPlay = rclcpp_action::ServerGoalHandle<LedsPlay>;
    using LedIndexes = hri_interfaces::msg::LedIndexes;
    using LedModes = hri_interfaces::msg::LedModes;

    //CUSTOM_ACTION_CPP_PUBLIC
    explicit LedsPlayActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("leds_play_action_server_node", options) {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<LedsPlay>(
                                   this,
                                   "leds_play",
                                   std::bind(&LedsPlayActionServer::handle_goal, this, _1, _2),
                                   std::bind(&LedsPlayActionServer::handle_cancel, this, _1),
                                   std::bind(&LedsPlayActionServer::handle_accepted, this, _1));

        this->head_pub_ = this->create_publisher<nao_lola_command_msgs::msg::HeadLeds>(
                              "effectors/head_leds", 10);
        this->right_eye_pub_ = this->create_publisher<nao_lola_command_msgs::msg::RightEyeLeds>(
                                   "effectors/right_eye_leds", 10);
        this->left_eye_pub_ = this->create_publisher<nao_lola_command_msgs::msg::LeftEyeLeds>(
                                  "effectors/left_eye_leds", 10);
        this->right_ear_pub_ = this->create_publisher<nao_lola_command_msgs::msg::RightEarLeds>(
                                   "effectors/right_ear_leds", 10);
        this->left_ear_pub_ = this->create_publisher<nao_lola_command_msgs::msg::LeftEarLeds>(
                                  "effectors/left_ear_leds", 10);
        this->chest_pub_ = this->create_publisher<nao_lola_command_msgs::msg::ChestLed>(
                               "effectors/chest_led", 10);
        this->right_foot_pub_ = this->create_publisher<nao_lola_command_msgs::msg::RightFootLed>(
                                    "effectors/right_foot_led", 10);
        this->left_foot_pub_ = this->create_publisher<nao_lola_command_msgs::msg::LeftFootLed>(
                                   "effectors/left_foot_led", 10);

        this->color_off_ = std_msgs::msg::ColorRGBA();
        this->color_off_.r=0.0;
        this->color_off_.g=0.0;
        this->color_off_.b=0.0;

        RCLCPP_INFO(this->get_logger(), "LedsPlayActionServer Initialized");

    }

  private:

    rclcpp_action::Server<LedsPlay>::SharedPtr action_server_;

    rclcpp::Publisher<nao_lola_command_msgs::msg::HeadLeds>::SharedPtr        head_pub_;
    rclcpp::Publisher<nao_lola_command_msgs::msg::RightEyeLeds>::SharedPtr    right_eye_pub_;
    rclcpp::Publisher<nao_lola_command_msgs::msg::LeftEyeLeds>::SharedPtr     left_eye_pub_;
    rclcpp::Publisher<nao_lola_command_msgs::msg::RightEarLeds>::SharedPtr    right_ear_pub_;
    rclcpp::Publisher<nao_lola_command_msgs::msg::LeftEarLeds>::SharedPtr     left_ear_pub_;
    rclcpp::Publisher<nao_lola_command_msgs::msg::ChestLed>::SharedPtr        chest_pub_;
    rclcpp::Publisher<nao_lola_command_msgs::msg::RightFootLed>::SharedPtr    right_foot_pub_;
    rclcpp::Publisher<nao_lola_command_msgs::msg::LeftFootLed>::SharedPtr     left_foot_pub_;

    std_msgs::msg::ColorRGBA color_off_; 
    //color_off_.r=0.0; color_off_.g=0.0; color_off_.b=0.0;
    hri_interfaces::msg::LedIndexes led_indexes_msg_;
    hri_interfaces::msg::LedModes led_modes_msg_;


    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const LedsPlay::Goal> goal) {

        RCLCPP_INFO( this->get_logger(), "Received goal request");
        (void)uuid;

        //TODO checks on goal integrity

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLedsPlay> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleLedsPlay> goal_handle) {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&LedsPlayActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleLedsPlay> goal_handle) {

        RCLCPP_INFO(this->get_logger(), "Executing goal");

        auto goal = goal_handle->get_goal();

        RCLCPP_INFO( this->get_logger(), "Executing goal 1"  );


        std::array<uint8_t, 2> leds = goal->leds;
        uint8_t mode = goal->mode;
        float frequency = goal->frequency;
        std::array<std_msgs::msg::ColorRGBA, 8> colors = goal->colors;
        std::array<float, 12> intensities = goal->intensities;
        float duration = goal->duration;

        RCLCPP_INFO_STREAM(this->get_logger(), "Executing goal 2" << led_modes_msg_.STATIC);

        nao_lola_command_msgs::msg::RightEyeLeds right_eye_leds;


        right_eye(goal_handle, mode, frequency, colors, duration);





        auto feedback = std::make_shared<LedsPlay::Feedback>();
        auto result = std::make_shared<LedsPlay::Result>();


        if (rclcpp::ok()) {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }

    }

    void right_eye( const std::shared_ptr<GoalHandleLedsPlay> goal_handle, uint8_t mode, float frequency,
                    std::array<std_msgs::msg::ColorRGBA, 8> & colors, float duration) {

        auto result = std::make_shared<LedsPlay::Result>();
        nao_lola_command_msgs::msg::RightEyeLeds right_eye_leds;

        if (mode == LedModes::STATIC) {
            for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEyeLeds::NUM_LEDS; ++i) {
                right_eye_leds.colors[i] = colors[i];
            }
            right_eye_pub_->publish(right_eye_leds);
        } else if (mode == LedModes::BLINKING) {

            rclcpp::Rate loop_rate(frequency);
            uint8_t c = 0;
            while (rclcpp::ok()) {
                if (goal_handle->is_canceling()) {
                    result->success = true;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                }
                c = (c + 1) % 2;
                for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEyeLeds::NUM_LEDS; ++i) {
                    if (c == 0) {
                        right_eye_leds.colors[i] = color_off_;
                    } else {
                        right_eye_leds.colors[i] = colors[i];
                    }
                }
                right_eye_pub_->publish(right_eye_leds);
                loop_rate.sleep();
            }


        } else { //LOOP

        }



        /*

        */

    }

};  // class LedsPlayActionServer

}  // namespace hri_led_action_server

RCLCPP_COMPONENTS_REGISTER_NODE(hri_led_action_server::LedsPlayActionServer)