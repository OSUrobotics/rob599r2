// fib_client2.cpp
//
// Bill Smart
//
// An example of a simple action client in ROS 2.

// Include the basic ROS functionality and message definitions.
#include <rclcpp/rclcpp.hpp>
#include "rob599_msgs/srv/fib_service.hpp"

// We're using chrono literals again.
#include <chrono>
using namespace std::chrono_literals;


// Build a class to encapsulate the functionality of the Fibbonacci client.  This
// is a different idiom that we usually see, since this class doesn't inherit from
// the node class.  Since we want this class to act like a function call, we don't
// want it to *be* the node.  It just needs to *use* the node.  So, we can get away
// with passing in a shared pointer to the node.  There are ways to do this while
// still using the more common idiom, but we'll use this as an example of explicitly
// not using it.
class Fibonacci {
public:
	Fibonacci(std::shared_ptr<rclcpp::Node> node) :node_(node) {
		// Make a client.  This is the only reason we need a node pointer.
		client_ = node_->create_client<rob599_msgs::srv::FibService>("fibonacci");
	}
	~Fibonacci() {}

	// A function the encapsulates the service call.  This will present an interface
	// that makes the class look like a function, which will make the code that uses
	// it tidier.
	long operator()(const int n) {
		// Make a request variable and fill in the argument.
		auto request = std::make_shared<rob599_msgs::srv::FibService::Request>();
		request->number = n;
		
		// Wait until the server is ready, as in the other client example.
		while (!client_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(node_->get_logger(), "Problem while waiting for server.");
				return 0;
			}

			RCLCPP_ERROR(node_->get_logger(), "No service.  Trying again.");
		}

		// Send the request, using an async call.
		auto result = client_->async_send_request(request);

		// Wait for the future and make sure it's successful.
		if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
			// Return the value.
			return result.get()->fibonacci;
		} else {
			RCLCPP_INFO(node_->get_logger(), "Service call failure.");

			// Since we have to return something, return an impossible value.  An alternative would
			// be to throw an exception here, which is probably a better design.
			return -1;
		}
	}

private:
	// Variables to store a pointer to the node and the client.
	std::shared_ptr<rclcpp::Node> node_;
	rclcpp::Client<rob599_msgs::srv::FibService>::SharedPtr client_;
};


// This is the entry point for the executable.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Make a node.
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("fib_client");

	// Make an instance of the Fibonacci class, and pass it the node we just made.
	Fibonacci fib(node);

	// Loop for some values
	for(int i = 0; i < 10; ++i)
		// Log the number and the corresponding Fibonacci number.  Note that the function uses the
		// class instance as a functor, as if it was actually a function.  This implicitly uses the
		// operator() class function we implemented above.
		RCLCPP_INFO(node->get_logger(), "%i  %li", i, fib(i));


	// Close everything down gracefully.
	rclcpp::shutdown();

	// And, we're done.
	return 0;
}
