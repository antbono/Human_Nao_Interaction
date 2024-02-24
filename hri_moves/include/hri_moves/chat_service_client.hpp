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

#ifndef HRI_MOVES__CHAT_SERVICE_CLIENT_HPP_
#define HRI_MOVES__CHAT_SERVICE_CLIENT_HPP_

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"

#include "hri_interfaces/srv/chat.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"



namespace hri_chat_service_client {


class ChatServiceClient : public rclcpp::Node {

  public:
  explicit ChatServiceClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~ChatServiceClient();

  std::string sendSyncReq(std::string & phrase);

  private:

  rclcpp::Client<hri_interfaces::srv::Chat>::SharedPtr client_ptr_;

};


}  // namespace hri_gstt_service_client

#endif //HRI_MOVES__CHAT_SERVICE_CLIENT_HPP_