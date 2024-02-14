// counter.cpp
//
// Bill Smart
//
// This is an example of a simple publisher in ROS 2.

// Include the basic ROS functionality.
#include <rclcpp/rclcpp.hpp>

// Include the definition of the int64 message type.
#include <std_msgs/msg/int64.hpp>

// Include the ability to use chrono_literals, so that our code looks prettier.
#include <chrono>

// Use the namespace, so that we don't have to namespace the literals.
using namespace std::chrono_literals;


// Creata a class that inherits from Node.
class CounterNode : public rclcpp::Node {
public:
	CounterNode() :Node("counter"), count_(0) {
		// Create a publisher.
		publisher_ = this->create_publisher<std_msgs::msg::Int64>("count", 10);

		// Create a timer to control the publication.
		timer_ = this->create_wall_timer(500ms, std::bind(&CounterNode::timer_callback, this));
	}

private:
	// A variable to hold the current count.
	long count_;

	// Variables for the publisher and timer.
	rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	// The callback that the timer uses.
	void timer_callback() {
		// Make a new message, and set the data element. We can use the auto
		// keyword here, since the type is clear.
		auto message = std_msgs::msg::Int64();
		message.data = count_++;

		// Publish the message.
		publisher_->publish(message);

		// Record in the log that we published the message.
		RCLCPP_INFO(this->get_logger(), "published %li", message.data);
	}
};


// This is the entry point of the executable.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node and give control to the ROS event handler.
	rclcpp::spin(std::make_shared<CounterNode>());

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	return 0;
}
