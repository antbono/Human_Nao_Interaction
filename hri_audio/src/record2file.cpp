#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
/*
bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
*/

using namespace std::chrono_literals;
using namespace std::placeholders;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Record2File : public rclcpp::Node {
  public:
	Record2File()
		: Node("record2file") {
		shutdown_pub_ = this->create_publisher<std_msgs::msg::Bool>("/audio/rec_shutdown", 10);
		rec_service_ = this->create_service<std_srvs::srv::SetBool>(
			"record_to_file", std::bind(&Record2File::record, this, _1, _2) );
	}

  private:

	void record(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
	            std::shared_ptr<std_srvs::srv::SetBool::Response>      response) {

		int sys_exit;
		if (system(NULL)) puts ("Ok");
		else exit (EXIT_FAILURE);

		if (request->data) {
			//printf ("Checking if processor is available...");
			//printf ("Executing command DIR...\n");
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request: START");
 //x-terminal-emulator -e "bash -c 'echo ciao; read'"

			sys_exit = system ("x-terminal-emulator -e \" bash -c 'source /opt/ros/rolling/setup.bash;\
						 source /home/toto/Gdrive/uni/robocup/robocup_ws/install/local_setup.bash;\
						  ros2 launch audio_capture capture_to_file.launch.xml; read' \" ");

			if (sys_exit == 0) {
				response->success = true;
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: recording node started");
			} else {
				response->success = false;
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: recording node PROBLEM");
			}
		} else {
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request: STOP");
			auto message = std_msgs::msg::Bool();
			message.data = true;
			//RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
			shutdown_pub_->publish(message);
			rclcpp::sleep_for(500ms);
			response->success = true;
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: recording node stopped");
		}
	}

	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr rec_service_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shutdown_pub_;

};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Record2File>());
	rclcpp::shutdown();
	return 0;
}