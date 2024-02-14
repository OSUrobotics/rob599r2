// params.cpp
//
// Bill Smart
//
// This is an example of using parameters in ROS 2.

// Include the basic ROS functionality.
#include <rclcpp/rclcpp.hpp>

// Include the definition of the int64 message type.
#include <std_msgs/msg/int64.hpp>

// We're going to use chrono literals, again.
#include <chrono>
using namespace std::chrono_literals;


// Creata a class that inherits from Node.
class ParamDemo : public rclcpp::Node {
public:
	ParamDemo() :Node("param_demo") {
		// Create a timer to control the publication.
		timer_ = this->create_wall_timer(1s, std::bind(&ParamDemo::timer_callback, this));

		// We need to declare that we're using a parameter.
		auto desc = rcl_interfaces::msg::ParameterDescriptor{};
		desc.description = "This parameter sets the robot speed.";
		this->declare_parameter("speed", 10.0, desc);
	}

	// This is a convenience function that allows the class to set the parameter value.
	void set_speed(const double speed) {
		// We have to construct a vector of Parameters.  In this case, there's only going
		// to be one.  Parameters have a key and a value.  The set_parameters function,
		// called on the node, actually sets the parameters.
	    std::vector<rclcpp::Parameter> parameter_values{rclcpp::Parameter("speed", speed)};
	    this->set_parameters(parameter_values);		
	}

private:
	// Variable for the timer.
	rclcpp::TimerBase::SharedPtr timer_;

	// The callback that the timer uses.
	void timer_callback() {
		// Polling for the value requires the name, and a call to a type-specific function.
		double value = this->get_parameter("speed").as_double();

		// Record in the log that we published the message.
		RCLCPP_INFO(this->get_logger(), "Timer: parameter is %f", value);
	}
};


// This is the entry point of the executable.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node and give control to the ROS event handler.
	rclcpp::spin(std::make_shared<ParamDemo>());

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	return 0;
}
