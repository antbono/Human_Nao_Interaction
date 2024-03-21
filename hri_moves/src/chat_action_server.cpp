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

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>


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
#include "hri_interfaces/srv/text_to_speech.hpp"
#include "hri_interfaces/action/joints_play.hpp"
#include "hri_interfaces/action/chat_play.hpp"


#include "hri_moves/chat_action_server.hpp"
#include "hri_moves/joints_play_action_client.hpp"
#include "hri_moves/led_action_server.hpp"


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_srvs/srv/set_bool.hpp"


namespace hri_chat_action_server {

using namespace std::chrono_literals;


ChatActionServer::ChatActionServer(const rclcpp::NodeOptions & options)
    : rclcpp::Node("chat_action_server_node", options), kSecPerWord_(0.5), kForwardParam_(5) {
    using namespace std::placeholders;


    this->gstt_srv_client_ = this->create_client<std_srvs::srv::SetBool>("gstt_service");
    this->gtts_srv_client_ = this->create_client<hri_interfaces::srv::TextToSpeech>("gtts_service");
    this->chat_srv_client_ = this->create_client<hri_interfaces::srv::Chat>("chatGPT_service");
    /*
    this->gstt_srv_client_ = std::make_shared<hri_gstt_service_client::GsttServiceClient>();
    this->gtts_srv_client_ = std::make_shared<hri_gtts_service_client::GttsServiceClient>();
    this->chat_srv_client_ = std::make_shared<hri_chat_service_client::ChatServiceClient>();
    this->led_srv_client_ = std::make_shared<hri_led_action_client::LedsPlayActionClient>();
    */

    /* LED ACTION CLIENT */
    //this->led_srv_client_ = std::make_shared<hri_led_action_client::LedsPlayActionClient>();
    this->leds_play_act_client_ = rclcpp_action::create_client<hri_interfaces::action::LedsPlay>(this,  "leds_play");
    //this->led_srv_client_ = std::make_shared<hri_led_action_client::LedsPlayActionClient>(this->leds_play_client_ptr_);


    /* JOINTS PLAY ACTION CLIENT */
    this->joints_act_client_ = rclcpp_action::create_client<hri_interfaces::action::JointsPlay>(
                                   this, "joints_play");

    //this->joints_play_client_ = std::make_shared<hri_joints_play_action_client::JointsPlayActionClient>();

    /* CHAT ACTION SERVER */
    this->action_server_ = rclcpp_action::create_server<hri_interfaces::action::ChatPlay>(
                               this,
                               "chat_play",
                               std::bind(&ChatActionServer::handleGoal, this, _1, _2),
                               std::bind(&ChatActionServer::handleCancel, this, _1),
                               std::bind(&ChatActionServer::handleAccepted, this, _1));

    /*moves_map_["ciao"] = "install/hri_moves/include/moves/hello.txt";
    moves_map_["tu"] = "install/hri_moves/include/moves/you.txt";
    moves_map_["te"] = "install/hri_moves/include/moves/you.txt";
    moves_map_["grande"] = "install/hri_moves/include/moves/big.txt";
    moves_map_["grandi"] = "install/hri_moves/include/moves/big.txt";
    moves_map_["piccolo"] = "install/hri_moves/include/moves/little.txt";
    moves_map_["piccoli"] = "install/hri_moves/include/moves/little.txt";
    moves_map_["piccola"] = "install/hri_moves/include/moves/little.txt";
    moves_map_["piccole"] = "install/hri_moves/include/moves/little.txt";
    moves_map_["sotto"] = "install/hri_moves/include/moves/down.txt";
    moves_map_["sopra"] = "install/hri_moves/include/moves/up.txt";
    moves_map_["destra"] = "install/hri_moves/include/moves/right.txt";
    moves_map_["sinistra"] = "install/hri_moves/include/moves/left.txt";
    moves_map_["paura"] = "install/hri_moves/include/moves/fear.txt";
    moves_map_["paure"] = "install/hri_moves/include/moves/fear.txt";
    moves_map_["spavento"] = "install/hri_moves/include/moves/fear.txt";
    moves_map_["spaventato"] = "install/hri_moves/include/moves/fear.txt";*/
    moves_map_["hello"] = "install/hri_moves/include/moves/hello.txt";
    moves_map_["hi"] = "install/hri_moves/include/moves/hello.txt";
    //moves_map_["you"] = "install/hri_moves/include/moves/you.txt";
    moves_map_["big"] = "install/hri_moves/include/moves/big.txt";
    moves_map_["little"] = "install/hri_moves/include/moves/little.txt";
    moves_map_["down"] = "install/hri_moves/include/moves/down.txt";
    moves_map_["up"] = "install/hri_moves/include/moves/up.txt";
    moves_map_["right"] = "install/hri_moves/include/moves/right.txt";
    moves_map_["left"] = "install/hri_moves/include/moves/left.txt";
    moves_map_["fear"] = "install/hri_moves/include/moves/fear.txt";
    moves_map_["scared"] = "install/hri_moves/include/moves/fear.txt";

    RCLCPP_INFO(this->get_logger(), "ChatActionServer Initialized");

}

ChatActionServer::~ChatActionServer() {}


rclcpp_action::GoalResponse ChatActionServer::handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const hri_interfaces::action::ChatPlay::Goal> goal) {

    RCLCPP_INFO(this->get_logger(), "Received chat goal request");
    (void)uuid;

    while (!gstt_srv_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for gstt_service. Exiting.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "gstt_service not available, waiting again...");
    }

    while (!gtts_srv_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for gtts_service. Exiting.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "gtts_service not available, waiting again...");
    }

    while (!chat_srv_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for chat_service. Exiting.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "chat_service not available, waiting again...");
    }

    while (!joints_act_client_->wait_for_action_server(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for joints_play action server. Exiting.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "joints_play action server not available, waiting again...");
    }


    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse ChatActionServer::handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<hri_interfaces::action::ChatPlay>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel chat goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ChatActionServer::handleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<hri_interfaces::action::ChatPlay>> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ChatActionServer::execute, this, _1), goal_handle} .detach();
}

void ChatActionServer::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<hri_interfaces::action::ChatPlay>> goal_handle) {
    using namespace std::placeholders;
    
    RCLCPP_INFO(this->get_logger(), "Executing chat goal");

    auto chat_goal = goal_handle->get_goal();
    auto chat_feedback = std::make_shared<hri_interfaces::action::ChatPlay::Feedback>();
    auto chat_result = std::make_shared<hri_interfaces::action::ChatPlay::Result>();

    auto gstt_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    gstt_request->data = true;

    unsigned num_words = 0;
    bool first_word = true;
    std::vector<std::string> words;
    std::vector<std::string> key_words;
    std::vector<double> key_words_time;
    bool playing = false;
    double last_action_time;
    std::string recognized_speach;
    std::string chatgpt_answer;
    std::string word;
    std::string user_input;
    std::string action_path;
    double t_start;
    double t_key_word;
    double t_word;
    double t_cur;
    double t_sleep;

    this->eyesStatic(true);
    this->chestStatic(true);

    while (rclcpp::ok()) {

        if (goal_handle->is_canceling()) {
            chat_result->success = true;
            goal_handle->canceled(chat_result);
            RCLCPP_INFO(this->get_logger(), "Chat Goal canceled");
            return;
        }

        // stt service
        RCLCPP_INFO(this->get_logger(), "Ready to listen");

        //ears static
        if (!this->leds_play_act_client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "ledsPlay Action server not available after waiting");
            rclcpp::shutdown();
        }


        auto goal_msg = hri_interfaces::action::LedsPlay::Goal();

        goal_msg.leds = {hri_interfaces::msg::LedIndexes::REAR, hri_interfaces::msg::LedIndexes::LEAR};
        goal_msg.mode = hri_interfaces::msg::LedModes::STEADY;
        for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEarLeds::NUM_LEDS; ++i) {
            goal_msg.intensities[i] = 0.0;
        }

        auto send_goal_options = rclcpp_action::Client<hri_interfaces::action::LedsPlay>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            std::bind(&ChatActionServer::ledsPlayGoalResponseCallback, this, _1);

        send_goal_options.feedback_callback =
            std::bind(&ChatActionServer::ledsPlayFeedbackCallback, this, _1, _2);

        send_goal_options.result_callback =
            std::bind(&ChatActionServer::ledsPlayResultCallback, this, _1);

        RCLCPP_INFO(this->get_logger(), "ledsPlay ears static Sending goal:" );

        auto goal_handle_future = leds_play_act_client_->async_send_goal(goal_msg, send_goal_options);

        // synchronous
        auto goal_handle = goal_handle_future.get();
        auto result_future = leds_play_act_client_->async_get_result(goal_handle);
        auto result = result_future.get();

        RCLCPP_INFO(this->get_logger(), "ledsPlay synchronous call done" );

        this->earsLoop(true);

        //TTS

        auto gstt_future = gstt_srv_client_->async_send_request(gstt_request);
        RCLCPP_INFO(this->get_logger(), "gstt request sent" );
        auto gstt_result = gstt_future.get();      // wait for the result
        RCLCPP_INFO(this->get_logger(), "gstt gets result" );

        this->earsLoop(false);
        recognized_speach = gstt_result.get()->message;
        RCLCPP_INFO(this->get_logger(), ("Recognized speech: " + recognized_speach).c_str());


        /*
        auto gstt_result = gstt_srv_client_->async_send_request(gstt_request);
        // Wait for the result.
        if ( rclcpp::spin_until_future_complete(this->get_node_base_interface(), gstt_result) ==
            rclcpp::FutureReturnCode::SUCCESS){
        //if ( rclcpp::spin_until_future_complete(this, gstt_result) == rclcpp::FutureReturnCode::SUCCESS ) {
            recognized_speach = gstt_result.get()->message;
            RCLCPP_INFO(this->get_logger(), ("Recognized speach: " + recognized_speach).c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call gstt_service");
            return;
        }
        */



        //recognized_speach = gstt_srv_client_->sendSyncReq();

        this->headLoop(true);




        // chatgpt service
        //chatgpt_answer = chat_srv_client_->sendSyncReq(recognized_speach);

        auto chat_request = std::make_shared<hri_interfaces::srv::Chat::Request>();
        chat_request->question = recognized_speach;
        auto chat_future = chat_srv_client_->async_send_request(chat_request);
        auto chat_result = chat_future.get(); // wait for the result
        this->headLoop(false);
        chatgpt_answer = chat_result.get()->answer;
        RCLCPP_INFO(this->get_logger(), ("chatGPT answer: " + chatgpt_answer).c_str());

        /*
        auto chat_request = std::make_shared<hri_interfaces::srv::Chat::Request>();
        chat_request->question = recognized_speach;
        auto chat_result = chat_srv_client_->async_send_request(chat_request);

        RCLCPP_DEBUG(this->get_logger(), "wait chatGPT answer..");
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), chat_result) ==
                rclcpp::FutureReturnCode::SUCCESS) {
            chatgpt_answer = chat_result.get()->answer;
            RCLCPP_INFO(this->get_logger(), ("chatGPT answer: " + chatgpt_answer).c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call chat_service");
            return;
        }
        */

        RCLCPP_DEBUG(this->get_logger(), "chat request completed");

        // text and timing analysis of the answer
        num_words = 0;
        words.clear();
        key_words.clear();
        std::stringstream ss(chatgpt_answer);
        first_word = true;

        while (ss >> word) {
            words.push_back(word);
        }

        for (std::string w : words) {
            num_words += 1;
            std::transform(w.begin(), w.end(), w.begin(),
            [](unsigned char c) { return std::tolower(c); });
            if (moves_map_.find(w) != moves_map_.end()) {
                key_words.push_back(w);
                if (num_words <= kForwardParam_ && first_word) {
                    key_words_time.push_back(0.1);
                    first_word = false;
                } else if (num_words <= kForwardParam_ && !first_word) {
                    key_words_time.push_back(num_words * kSecPerWord_);
                } else {
                    key_words_time.push_back((num_words - kForwardParam_)*kSecPerWord_);
                }
            }

        }

        // speaking TTS
        this->earsStatic(true);

        //gtts_srv_client_->sendSyncReq(chatgpt_answer);

        auto gtts_request = std::make_shared<hri_interfaces::srv::TextToSpeech::Request>();
        gtts_request->text = chatgpt_answer;
        auto gtts_future = gtts_srv_client_->async_send_request(gtts_request);

        auto gtts_result = gtts_future.get(); // Wait for the result.
        if (gtts_result.get()->success) {
            RCLCPP_INFO(this->get_logger(), "tts request completed: %d", gtts_result.get()->success);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call gtts_service");
            return;
        }

        /*
        auto gtts_request = std::make_shared<hri_interfaces::srv::TextToSpeech::Request>();
        gtts_request->text = chatgpt_answer;
        auto gtts_result = gtts_srv_client_->async_send_request(gtts_request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), gtts_result) ==
                rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "tts request completed: %d", gtts_result.get()->success);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call gstt_service");
            return;
        }
        */
        //play moves
        t_start = this->now().seconds();
        //rclcpp::sleep_for(std::chrono::milliseconds(50));


        for (unsigned i = 0; i < key_words.size(); ++i) {

            t_key_word = key_words_time[i];
            t_word = t_key_word + t_start;
            t_cur = this->now().seconds();

            if (t_cur < t_word) {
                //wait
                t_sleep = t_word - t_cur;
                rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<uint64_t>(t_sleep * 1e9)));

                //execute move
                action_path = moves_map_[key_words[i]];

                auto goal_msg = hri_interfaces::action::JointsPlay::Goal();
                goal_msg.path = action_path;

                auto send_goal_options = rclcpp_action::Client<hri_interfaces::action::JointsPlay>::SendGoalOptions();

                send_goal_options.goal_response_callback =
                    std::bind(&ChatActionServer::jointsPlayGoalResponseCallback, this, std::placeholders::_1);
                //send_goal_options.feedback_callback =
                //    std::bind(&ChatActionServer::jointsPlayFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
                send_goal_options.result_callback =
                    std::bind(&ChatActionServer::jointsPlayResultCallback, this, std::placeholders::_1);

                RCLCPP_INFO(this->get_logger(), (" jointsPlay Sending goal: " + action_path).c_str() );

                this->joints_act_client_->async_send_goal(goal_msg, send_goal_options);

            }

        }


        /*
        for (unsigned i = 0; i < key_words.size(); ++i) {

            t_key_word = key_words_time[i];
            t_word = t_key_word + t_start;
            t_cur = this->now().seconds();

            if (t_cur < t_word) {
                //wait
                t_sleep = t_word - t_cur;
                rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<uint64_t>(t_sleep * 1e9)));

                //execute move
                action_path = moves_map_[key_words[i]];

                RCLCPP_INFO(this->get_logger(), ("Sending goal: " + action_path).c_str() );
                joints_play_client_->sendAsyncGoal(action_path);

                /*
                auto goal_msg = hri_interfaces::action::JointsPlay::Goal();
                goal_msg.path = action_path;

                auto send_goal_options = rclcpp_action::Client<hri_interfaces::action::JointsPlay>::SendGoalOptions();

                send_goal_options.goal_response_callback =
                    std::bind(&ChatActionServer::jointsPlayGoalRe sponseCallback, this, std::placeholders::_1);
                send_goal_options.feedback_callback =
                    std::bind(&ChatActionServer::jointsPlayFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
                send_goal_options.result_callback =
                    std::bind(&ChatActionServer::jointsPlayResultCallback, this, std::placeholders::_1);

                RCLCPP_INFO(this->get_logger(), ("Sending goal: " + action_path).c_str() );

                this->joints_act_client_->async_send_goal(goal_msg, send_goal_options);
                end comment!

            }

        }*/

        //user input
        std::cout << "Press after your listening is finished." << std::endl;
        std::getline(std::cin, user_input);

    }//execute while(rclcpp::ok())

}// execute

/*############## LEDS PLAY ACTION CLIENT ##############*/

void ChatActionServer::ledsPlayGoalResponseCallback(
    const rclcpp_action::ClientGoalHandle<hri_interfaces::action::LedsPlay>::SharedPtr & goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "ledsPlay Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "ledsPlay Goal accepted by server, waiting for result");
    }
}
void ChatActionServer::ledsPlayFeedbackCallback(
    rclcpp_action::ClientGoalHandle<hri_interfaces::action::LedsPlay>::SharedPtr,
    const std::shared_ptr<const hri_interfaces::action::LedsPlay::Feedback> feedback) {

}

void ChatActionServer::ledsPlayResultCallback(
    const rclcpp_action::ClientGoalHandle<hri_interfaces::action::LedsPlay>::WrappedResult & result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "ledsPlay Goal was aborted");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "ledsPlay Goal was canceled");
        return;
    default:
        RCLCPP_ERROR(this->get_logger(), "ledsPlay Unknown result code");
        return;
    }

    if (result.result->success)
        RCLCPP_INFO(this->get_logger(), "Leds regulary played.");

}

void  ChatActionServer::eyesStatic( bool flag ) {

    using namespace std::placeholders;
    if (!this->leds_play_act_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "ledsPlay Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = hri_interfaces::action::LedsPlay::Goal();

    goal_msg.leds = {hri_interfaces::msg::LedIndexes::REYE, hri_interfaces::msg::LedIndexes::LEYE};
    goal_msg.mode = hri_interfaces::msg::LedModes::STEADY;
    std_msgs::msg::ColorRGBA color;
    if (flag) {
        color.r = 1.0; color.g = 1.0; color.b = 1.0;
    } else {
        color.r = 0.0; color.g = 0.0; color.b = 0.0;
    }

    for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEyeLeds::NUM_LEDS; ++i) {
        goal_msg.colors[i] = color;
    }

    auto send_goal_options = rclcpp_action::Client<hri_interfaces::action::LedsPlay>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&ChatActionServer::ledsPlayGoalResponseCallback, this, _1);

    //send_goal_options.feedback_callback =
    //    std::bind(&ChatActionServer::ledsPlayFeedbackCallback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&ChatActionServer::ledsPlayResultCallback, this, _1);

    RCLCPP_INFO(this->get_logger(), "ledsPlay eyesStatic Sending goal:" );

    auto goal_handle_future = leds_play_act_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "ledsPlay eyesStatic goal sent: %d", flag);
    // synchronous
    //auto goal_handle = goal_handle_future.get();
    //auto result_future = leds_play_act_client_->async_get_result(goal_handle);
    //auto result = result_future.get();
}

void  ChatActionServer::chestStatic( bool flag ) {

    using namespace std::placeholders;
    if (!this->leds_play_act_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "ledsPlay Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = hri_interfaces::action::LedsPlay::Goal();

    goal_msg.leds = {hri_interfaces::msg::LedIndexes::CHEST};
    goal_msg.mode = hri_interfaces::msg::LedModes::STEADY;
    std_msgs::msg::ColorRGBA color;
    if (flag) {
        color.r = 1.0; color.g = 1.0; color.b = 1.0;
    } else {
        color.r = 0.0; color.g = 0.0; color.b = 0.0;
    }

    goal_msg.colors[0] = color;

    auto send_goal_options = rclcpp_action::Client<hri_interfaces::action::LedsPlay>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&ChatActionServer::ledsPlayGoalResponseCallback, this, _1);

    //send_goal_options.feedback_callback =
    //    std::bind(&ChatActionServer::ledsPlayFeedbackCallback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&ChatActionServer::ledsPlayResultCallback, this, _1);

    RCLCPP_INFO(this->get_logger(), "ledsPlay chestStatic Sending goal:" );

    auto goal_handle_future = this->leds_play_act_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "ledsPlay chestStatic goal sent: %d", flag);
    // synchronous
    //auto goal_handle = goal_handle_future.get();
    //auto result_future = leds_play_act_client_->async_get_result(goal_handle);
    //auto result = result_future.get();

}

void  ChatActionServer::headStatic(bool flag) {
    using namespace std::placeholders;

    if (!this->leds_play_act_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "ledsPlay Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = hri_interfaces::action::LedsPlay::Goal();

    goal_msg.leds = {hri_interfaces::msg::LedIndexes::HEAD};
    goal_msg.mode = hri_interfaces::msg::LedModes::STEADY;
    for (unsigned i = 0; i < nao_lola_command_msgs::msg::HeadLeds::NUM_LEDS; ++i) {
        if (flag) {
            goal_msg.intensities[i] = 1.0;
        } else {goal_msg.intensities[i] = 0.0;}
    }

    auto send_goal_options = rclcpp_action::Client<hri_interfaces::action::LedsPlay>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&ChatActionServer::ledsPlayGoalResponseCallback, this, _1);

    //send_goal_options.feedback_callback =
    //    std::bind(&ChatActionServer::ledsPlayFeedbackCallback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&ChatActionServer::ledsPlayResultCallback, this, _1);


    auto goal_handle_future = this->leds_play_act_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "ledsPlay headStatic goal sent: " );
    // synchronous
    //auto goal_handle = goal_handle_future.get();
    //auto result_future = leds_play_act_client_->async_get_result(goal_handle);
    //auto result = result_future.get();

}

void  ChatActionServer::earsStatic(bool flag) {
    using namespace std::placeholders;

    if (!this->leds_play_act_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "ledsPlay Action server not available after waiting");
        rclcpp::shutdown();
    }


    auto goal_msg = hri_interfaces::action::LedsPlay::Goal();

    goal_msg.leds = {hri_interfaces::msg::LedIndexes::REAR, hri_interfaces::msg::LedIndexes::LEAR};
    goal_msg.mode = hri_interfaces::msg::LedModes::STEADY;
    for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEarLeds::NUM_LEDS; ++i) {
        if (flag) {
            goal_msg.intensities[i] = 1.0;
        } else {goal_msg.intensities[i] = 0.0;}
    }

    auto send_goal_options = rclcpp_action::Client<hri_interfaces::action::LedsPlay>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&ChatActionServer::ledsPlayGoalResponseCallback, this, _1);

    //send_goal_options.feedback_callback =
    //    std::bind(&ChatActionServer::ledsPlayFeedbackCallback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&ChatActionServer::ledsPlayResultCallback, this, _1);

    RCLCPP_INFO(this->get_logger(), "ledsPlay earsStatic Sending goal:" );

    auto goal_handle_future = leds_play_act_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "ledsPlay earsStatic goal sent: %d", flag);


    // synchronous
    //auto goal_handle = goal_handle_future.get();
    //auto result_future = leds_play_act_client_->async_get_result(goal_handle);
    //auto result = result_future.get();

}

void ChatActionServer::headLoop( bool flag )  {
    using namespace std::placeholders;

    if (!this->leds_play_act_client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "ledsPlay Action server not available after waiting");
        rclcpp::shutdown();
    }

    if (flag) {
        auto goal_msg = hri_interfaces::action::LedsPlay::Goal();

        goal_msg.leds = {hri_interfaces::msg::LedIndexes::HEAD};
        goal_msg.mode = hri_interfaces::msg::LedModes::LOOP;
        for (unsigned i = 0; i < nao_lola_command_msgs::msg::HeadLeds::NUM_LEDS; ++i) {
            goal_msg.intensities[i] = 1.0;
        }
        goal_msg.frequency = 10.0;

        auto send_goal_options = rclcpp_action::Client<hri_interfaces::action::LedsPlay>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            std::bind(&ChatActionServer::ledsPlayGoalResponseCallback, this, _1);

    //    send_goal_options.feedback_callback =
    //        std::bind(&ChatActionServer::ledsPlayFeedbackCallback, this, _1, _2);

        send_goal_options.result_callback =
            std::bind(&ChatActionServer::ledsPlayResultCallback, this, _1);

        RCLCPP_INFO(this->get_logger(), "ledsPlay headLoop Sending goal:" );

        auto goal_handle_future = leds_play_act_client_->async_send_goal(goal_msg, send_goal_options);
        head_goal_handle_ = goal_handle_future.get();
        //head_goal_handle_ = client_ptr_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "ledsPlay headLoop gets goal handle:" );
    } else {
        //rclcpp_action::ClientGoalHandle<hri_interfaces::action::LedsPlay>::SharedPtr handle = head_goal_handle_.get();
        //auto cancel_result_future = client_ptr_->async_cancel_goal(handle);
        auto cancel_result_future = leds_play_act_client_->async_cancel_goal(head_goal_handle_);
        auto cancel_result = cancel_result_future.get(); //wait
        RCLCPP_INFO(this->get_logger(), "ledsPlay headLoop cancel gets result");

        this->headStatic(false);
    }
}

void ChatActionServer::earsLoop(bool flag)  {
    using namespace std::placeholders;

    if (flag) {
        if (!this->leds_play_act_client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "ledsPlay Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = hri_interfaces::action::LedsPlay::Goal();

        goal_msg.leds = {hri_interfaces::msg::LedIndexes::REAR, hri_interfaces::msg::LedIndexes::LEAR};
        goal_msg.mode = hri_interfaces::msg::LedModes::LOOP;
        for (unsigned i = 0; i < nao_lola_command_msgs::msg::RightEarLeds::NUM_LEDS; ++i) {
            goal_msg.intensities[i] = 1.0;
        }
        goal_msg.frequency = 10.0;

        auto send_goal_options = rclcpp_action::Client<hri_interfaces::action::LedsPlay>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            std::bind(&ChatActionServer::ledsPlayGoalResponseCallback, this, _1);

        //send_goal_options.feedback_callback =
        //    std::bind(&ChatActionServer::ledsPlayFeedbackCallback, this, _1, _2);

        send_goal_options.result_callback =
            std::bind(&ChatActionServer::ledsPlayResultCallback, this, _1);

        RCLCPP_INFO(this->get_logger(), "ledsPlay earsLoop Sending goal:" );

        auto goal_handle_future = leds_play_act_client_->async_send_goal(goal_msg, send_goal_options);
        ears_goal_handle_ = goal_handle_future.get();
        RCLCPP_INFO(this->get_logger(), "ledsPlay earsLoop gets goal handle:" );
    } else {
        auto cancel_result_future = leds_play_act_client_->async_cancel_goal(ears_goal_handle_);
        auto cancel_result = cancel_result_future.get(); //wait
        RCLCPP_INFO(this->get_logger(), "ledsPlay earsLoop cancel gets result");
        this->earsStatic(false);
    }
}

/*############## JOINTS PLAY ACTION CLIENT ##############*/

void ChatActionServer::jointsPlayGoalResponseCallback(
    const rclcpp_action::ClientGoalHandle<hri_interfaces::action::JointsPlay>::SharedPtr & goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "jointsPlay goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "jointsPlay goal accepted by server, waiting for result");
    }
}

void ChatActionServer::jointsPlayFeedbackCallback(
    rclcpp_action::ClientGoalHandle<hri_interfaces::action::JointsPlay>::SharedPtr,
    const std::shared_ptr<const hri_interfaces::action::JointsPlay::Feedback> feedback) {

    //TODO

}

void ChatActionServer::jointsPlayResultCallback(
    const rclcpp_action::ClientGoalHandle<hri_interfaces::action::JointsPlay>::WrappedResult & result) {

    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), " jointsPlay Goal was aborted");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "jointsPlay Goal was canceled");
        return;
    default:
        RCLCPP_ERROR(this->get_logger(), "jointsPlay Unknown result code");
        return;
    }

    if (result.result->success)
        RCLCPP_INFO(this->get_logger(), "Joints posisitions regulary played.");

}


}  // namespace hri_chat_action_server

RCLCPP_COMPONENTS_REGISTER_NODE(hri_chat_action_server::ChatActionServer)