// node.cpp
//
// Bill Smart
//
// This is an example of a minimal node in ROS 2.

// Include the basic ROS 2 stuff.
#include <rclcpp/rclcpp.hpp>


// The idiom in C++ is the same as in Python; we create a class that
// inherits from the ROS 2 Node class.
class MinimalNode : public rclcpp::Node {
public:
	MinimalNode() :Node("minimal_node") {
		// Send a log message to let people know we're running.
		RCLCPP_INFO(this->get_logger(), "MinimalNode started");
	}
};


// This is the entry point for the executable.  Unlike Python, we
// can only have a single entry point in C++.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node and give control to the ROS event handler.
	rclcpp::spin(std::make_shared<MinimalNode>());

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	return 0;
}