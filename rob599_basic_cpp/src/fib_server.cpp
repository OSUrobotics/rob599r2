// fib_server.cpp
//
// Bill Smart
//
// An example of a simple service server in ROS 2.

// Include the basic ROS functionality.
#include <rclcpp/rclcpp.hpp>

// Include our custom message type definition.
#include "rob599_msgs/srv/fib_service.hpp"

// The service callback is going to need two parameters, so we declare that
// we're going to use two placeholders.
using std::placeholders::_1;
using std::placeholders::_2;


// Create a class that inherits from Node.
class FibServerNode : public rclcpp::Node {
public:
	FibServerNode() :Node("fib_server") {
		// Set up a service server.  This callback takes two arguments, so we
		// use two placeholders.
		service_ = this->create_service<rob599_msgs::srv::FibService>("fibonacci", std::bind(&FibServerNode::fibonacci, this, _1, _2));

		RCLCPP_INFO(this->get_logger(), "Fibonacci server ready");
	}

private:
	// A variable for the service.
	rclcpp::Service<rob599_msgs::srv::FibService>::SharedPtr service_;

	// Define a function to do the legwork of computing the Fibonacci number.
	// Since it doesn't depend on any information in the class, we'll make it
	// a static.
	static long fib(const int n) {
		if (n < 0)
			return 0;
		else if (n < 2)
			return n;
		else
			return fib(n - 1) + fib(n - 2);
	}

	// This is the callback function.  It takes a request and a response.  The
	// callback returns void; results are returned through the response
	// argument.
	void fibonacci(const std::shared_ptr<rob599_msgs::srv::FibService::Request> request,
		const std::shared_ptr<rob599_msgs::srv::FibService::Response> response) const {

		// The result is returned by filling in the response.
		response->fibonacci = fib(request->number);
	}
};


// This is the entry point for the executable.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node and give control to the ROS event handler.
	rclcpp::spin(std::make_shared<FibServerNode>());

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	return 0;
}