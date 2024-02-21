// cloud_timer_node.cpp
//
// Bill Smart
//
// The implementaiton of the CloudTimerNode


#include "cloud_timer_node.hpp"


// Some C++ stuff that we're going to need.
#include <vector>
#include <numeric>

// We're going to use a callback with one parameter.
using std::placeholders::_1;


CloudTimerNode::CloudTimerNode() :Node("cloud_timer"), counter_(0) {
	// A subscriber for the point clouds.
	subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("clouds", 10, std::bind(&CloudTimerNode::callback, this, _1));

	// Two publishers for topics where we'll publish latencies.
	publisher_ = this->create_publisher<std_msgs::msg::Float32>("latency", 10);
	average_publisher_ = this->create_publisher<std_msgs::msg::Float32>("average_latency", 10);
}


CloudTimerNode::~CloudTimerNode() {
}


void CloudTimerNode::CloudTimerNode::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
	// Get the time difference
	double difference = double((this->get_clock()->now() - msg->header.stamp).nanoseconds()) / 1e6;

	// Add to the buffer of last buffer_size values.  If we don't have 100 values yet, just add them
	// to the vector.  Otherwise, replace on the of the existing values.
	if (buffer_.size() < buffer_size_) {
		buffer_.push_back(difference);
	} else {
		buffer_[counter_] = difference;
		counter_ = (counter_ + 1) % buffer_size_;
	}

	// Get the running average of up to the last ten values
	double average = std::reduce(buffer_.begin(), buffer_.end()) / buffer_.size();

	// Publish the current latency.
	auto latency_msg = std_msgs::msg::Float32();
	latency_msg.data = difference;
	publisher_->publish(latency_msg);

	// Publish the average latency.  Reuse the existing message.
	latency_msg.data = average;
	average_publisher_->publish(latency_msg);

	if (buffer_.size() == buffer_size_)
		RCLCPP_INFO(this->get_logger(), "100!");

	RCLCPP_INFO(this->get_logger(), "Got a point cloud (size %li): latency is %fms, average: %fms", msg->data.size(), difference, average);
}
