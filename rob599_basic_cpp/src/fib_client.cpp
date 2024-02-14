// fib_client.cpp
//
// Bill Smart
//
// An example of a simple C++ service client.

// Include basic ROS functionality and service message definitions.
#include <rclcpp/rclcpp.hpp>
#include "rob599_msgs/srv/fib_service.hpp"

// We're going to use chrono literals to make our code pretty.
#include <chrono>
using namespace std::chrono_literals;


// This is the entry point for the executable.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// We still need to make a node, even if we're not inheriting from one.
	// We're not using the idiom of inheriting from the Node class for this
	// example because this node is not meant to keep running.  It has a
	// limited execuation, where it makes a number of service calls and then
	// ends.
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("fib_client");

	// Create a service client.
	rclcpp::Client<rob599_msgs::srv::FibService>::SharedPtr client = node->create_client<rob599_msgs::srv::FibService>("fibonacci");

	// Build a request variable.  Since the type is discoverable, we can use the
	// auto keyword.
	auto request = std::make_shared<rob599_msgs::srv::FibService::Request>();

	// Make a bunch of calls
	for(int n = 0; n < 10; ++n) {
		// Set the data field in the request variable.
		request->number = n;

		// Wait until the service server is ready for us.  Wait for 1s, then print a message saying
		// that we're waiting.  During this look, check for a shutdown.  If there is one, then end
		// the program.
		while (!client->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(node->get_logger(), "Problem while waiting for server.");
				return 0;
			}

			RCLCPP_ERROR(node->get_logger(), "No service.  Trying again.");
		}

		// Send a request to the service server.  This call is async.  This is called a future.  We
		// use the auto keyword to create a variable that holds a bunch of information about the
		// async call.  We can use this info to see if the call succeeded and get the response value
		// from it.
		auto result = client->async_send_request(request);

		// The spin_until_future_complete waits until we get a result from the service server.  If
		// the return value indicates success, then we can get the response from the result future.
		if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
			// Calling get() on the future lets us access the response type through a pointer.
			RCLCPP_INFO(node->get_logger(), "%i -> %li", n, result.get()->fibonacci);
		} else {
			RCLCPP_INFO(node->get_logger(), "Service call failure.");
		}
	}

	// And, we're done.
	rclcpp::shutdown();

	return 0;
}