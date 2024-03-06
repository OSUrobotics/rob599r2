#ifndef EXAMPLE_BT_NODES_HPP
#define EXAMPLE_BT_NODES_HPP

#include <iostream>
#include <random>

#include <chrono>
using namespace std::chrono_literals;

#include <behaviortree_cpp/behavior_tree.h>

using namespace BT;


// Set up random number generation.
std::random_device dev;
std::mt19937 rng(dev());
std::uniform_int_distribution<std::mt19937::result_type> dist(0, 4);

// Set the distance to a simulated object to a random value.
int distance = dist(rng);


class Speak : public SyncActionNode {
public:
	Speak(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config) {}

	static PortsList providedPorts() {
		return {InputPort<std::string>("message")};
	}

	NodeStatus tick() override {
		std::string message;
		getInput("message", message);
		std::cout << message << std::endl;
		return NodeStatus::SUCCESS;
	}
};


class Find : public SyncActionNode {
public:
	Find(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config) {}

	static PortsList providedPorts() {
		return {};
	}

	NodeStatus tick() override {
		std::cout << "Find" << std::endl;
		return NodeStatus::SUCCESS;
	}
};


class Pick : public SyncActionNode {
public:
	Pick(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config) {}

	static PortsList providedPorts() {
		return {};
	}

	NodeStatus tick() override {
		std::cout << "Pick" << std::endl;
		return NodeStatus::SUCCESS;
	}
};


class Place : public SyncActionNode {
public:
	Place(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config) {}

	static PortsList providedPorts() {
		return {};
	}

	NodeStatus tick() override {
		std::cout << "Place" << std::endl;
		return NodeStatus::SUCCESS;
	}
};


class Failure : public SyncActionNode {
public:
	Failure(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config) {}

	static PortsList providedPorts() {
		return {};
	}

	NodeStatus tick() override {
		std::cout << "Failure!" << std::endl;
		return NodeStatus::SUCCESS;
	}
};


class MoveCloser : public SyncActionNode {
public:
	MoveCloser(const std::string &name, const NodeConfiguration &config) : SyncActionNode(name, config) {}

	static PortsList providedPorts() {
		return {};
	}

	NodeStatus tick() override {
		std::cout << "Moving closer: ";

		bool close = (distance == 0);

		if (close) {
			std::cout << "got there!" << std::endl;
			return NodeStatus::SUCCESS;			
		} else {
			std::cout << "not there yet (" << distance << ")" << std::endl;
			distance -= 1;
			return NodeStatus::FAILURE;
		}
	}
};


NodeStatus ThingClose() {
	bool close = (distance == 0);

	std::cout << "Thing is ";

	if (close) {
		std::cout << "close" << std::endl;
		return NodeStatus::SUCCESS;
	} else {
		std::cout << "not close" << std::endl;
		return NodeStatus::FAILURE;
	}
}


// Largely copied from https://www.behaviortree.dev/docs/tutorial-basics/tutorial_04_sequence
class MoveBaseAction : public StatefulActionNode {
  public:
    MoveBaseAction(const std::string &name, const BT::NodeConfig &config) :StatefulActionNode(name, config) {}

    static PortsList providedPorts() {
        return{InputPort<std::string>("goal")};
    }

    // Executed once at the beginning.
    NodeStatus onStart() override {
    	getInput<std::string>("goal", goal_);

    	std::cout << "Heading for: " << goal_ << std::endl;

    	// Simulate an event that takes some time to complete by setting an arbitrary time when it's going
    	// to be done.
    	completion_time_ = std::chrono::system_clock::now() + 321ms;

    	return NodeStatus::RUNNING;
    }

    // If onStart() returns RUNNING, repeatedly call this function until it returns something that it not
    // RUNNING.
    NodeStatus onRunning() override {
    	// Are we done?
    	if (std::chrono::system_clock::now() >= completion_time_) {
    		std::cout << "Done" << std::endl;
    		return NodeStatus::SUCCESS;
		} else {
    		std::cout << "Still working" << std::endl;
    		return NodeStatus::RUNNING;
    	}
    }

    // Executed if we're aborted by another BT node.
    void onHalted() override {
    }

  private:
  	std::string goal_;
  	std::chrono::system_clock::time_point completion_time_;
};


#endif
