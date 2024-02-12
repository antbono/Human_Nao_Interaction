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

#ifndef HRI_MOVES__CHAT_ACTION_SERVER_HPP_
#define HRI_MOVES__CHAT_ACTION_SERVER_HPP_

#include <array>
#include <cstring>
#include <functional>
#include <iostream>   // std::cout
#include <memory>
#include <string>     // std::string, std::stof
#include <thread>
#include <unordered_map>
//#include <vector>

//#include <chrono>
#include "hri_interfaces/action/joints_play.hpp"
#include "hri_interfaces/action/chat_play.hpp"
#include "hri_interfaces/srv/chat.hpp"
#include "hri_interfaces/srv/text_to_speech.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_srvs/srv/set_bool.hpp"


namespace hri_chat_action_server {


class ChatActionServer : public rclcpp::Node {
 public:
	explicit ChatActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
	virtual ~ChatActionServer();

 private:

	// services clients
	rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr gstt_srv_client_;
	rclcpp::Client<hri_interfaces::srv::TextToSpeech>::SharedPtr gtts_srv_client_;
	rclcpp::Client<hri_interfaces::srv::Chat>::SharedPtr chat_srv_client_;

	// chat play action server
	rclcpp_action::Server<hri_interfaces::action::ChatPlay>::SharedPtr action_server_;
	std::shared_ptr<rclcpp_action::ServerGoalHandle<hri_interfaces::action::ChatPlay>> goal_handle_;
	rclcpp_action::GoalResponse handleGoal(
	  const rclcpp_action::GoalUUID & uuid,
	  std::shared_ptr<const hri_interfaces::action::ChatPlay::Goal> goal);
	rclcpp_action::CancelResponse handleCancel(
	  const std::shared_ptr<rclcpp_action::ServerGoalHandle<hri_interfaces::action::ChatPlay>> goal_handle);
	void handleAccepted(
	  const std::shared_ptr<rclcpp_action::ServerGoalHandle<hri_interfaces::action::ChatPlay>> goal_handle);
	void execute(
	  const std::shared_ptr<rclcpp_action::ServerGoalHandle<hri_interfaces::action::ChatPlay>> goal_handle);


	// joints play action client
	rclcpp_action::Client<hri_interfaces::action::JointsPlay>::SharedPtr joints_act_client_;
	void jointsPlayGoalResponseCallback(
	  const rclcpp_action::ClientGoalHandle<hri_interfaces::action::JointsPlay>::SharedPtr & goal_handle);
	void jointsPlayFeedbackCallback(
	  rclcpp_action::ClientGoalHandle<hri_interfaces::action::JointsPlay>::SharedPtr,
	  const std::shared_ptr<const hri_interfaces::action::JointsPlay::Feedback> feedback);
	void jointsPlayResultCallback(
    const rclcpp_action::ClientGoalHandle<hri_interfaces::action::JointsPlay>::WrappedResult & result);

  // parameters
	const double kSecPerWord_ ;
	const double kForwardParam_ ;

	static std::unordered_map<std::string,std::string> moves_map_;

};

}

#endif  // HRI_MOVES__CHAT_ACTION_SERVER_HPP_