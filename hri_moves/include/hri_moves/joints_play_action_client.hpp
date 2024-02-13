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

#ifndef HRI_MOVES__JOINTS_PLAY_ACTION_SERVER_HPP_
#define HRI_MOVES__JOINTS_PLAY_ACTION_SERVER_HPP_

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>



#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"
#include "hri_interfaces/action/joints_play.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace fs = boost::filesystem;


namespace hri_joints_play_action_client {

using JointsPlay = hri_interfaces::action::JointsPlay;
using GoalHandleJointsPlay = rclcpp_action::ClientGoalHandle<JointsPlay>;

class JointsPlayActionClient: public rclcpp::Node {

  public:
	explicit JointsPlayActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
	virtual ~JointsPlayActionClient();

	void sendGoal(std::string & action_path);
	//void sendGoal();

  private:

  	rclcpp_action::Client<JointsPlay>::SharedPtr client_ptr_;
  	rclcpp::TimerBase::SharedPtr timer_;

  	std::string getDefaultFullFilePath();
  	void goalResponseCallback(const GoalHandleJointsPlay::SharedPtr & goal_handle);
  	void feedbackCallback( GoalHandleJointsPlay::SharedPtr,
    	const std::shared_ptr<const JointsPlay::Feedback> feedback);
    void resultCallback(const GoalHandleJointsPlay::WrappedResult & result);

};

} // namespace hri_joints_play_action_client


#endif //HRI_MOVES__JOINTS_PLAY_ACTION_SERVER_HPP_