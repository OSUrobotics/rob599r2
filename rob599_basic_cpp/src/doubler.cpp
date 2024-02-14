// doubler.cpp
//
// Bill Smart
//
// An example of a simple transformer node in ROS 2.  This node subscribes
// to a topic with int64 messages, doubles the value, and republishes.

// Include the basic ROS functionality and the message definition.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

// We need this in order to set up the subscriber.  This is the C++ way of
// letting us specify callback parameters.
using std::placeholders::_1;


// As usual, we create a class that inherits from Node.
class DoublerNode : public rclcpp::Node {
public:
	DoublerNode() :Node("doubler") {
		// Set up the publisher before the subscriber
		publisher_ = this->create_publisher<std_msgs::msg::Int64>("doubled", 10);

		// Set up the subscriber.  The callback argument uses std::bind to combine a
		// pointer to the callback and a placeholder for the single argument.
		subscriber_ = this->create_subscription<std_msgs::msg::Int64>("number", 10, std::bind(&DoublerNode::callback, this, _1));
	}

private:
	// Variables for the publisher and subscriber.
	rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
	rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;

	// The callback for the subscriber.  The message is passed via a shared
	// pointer.  This means you have to use the -> accessor, rather than the
	// . (dot) accessor.
	void callback(const std_msgs::msg::Int64::SharedPtr msg) const {
		// The type is clear, so we can use the auto keyword.
		auto doubled = std_msgs::msg::Int64();
		doubled.data = msg->data * 2;

		// Publish the message.
		publisher_->publish(doubled);

		// Write to the log.
		RCLCPP_INFO(this->get_logger(), "got %li and published %li", msg->data, doubled.data);
	}
};


int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node and give control to the ROS event handler.
	rclcpp::spin(std::make_shared<DoublerNode>());

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	return 0;
}