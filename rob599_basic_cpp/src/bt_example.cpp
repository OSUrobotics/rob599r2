#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include "example_bt_nodes.hpp"

using namespace BT;


int main(int argc, char **argv) {
	if (argc != 2) {
		std::cout << "Need a single filename!" << std::endl;
		return -1;
	}

	BehaviorTreeFactory factory;

	factory.registerNodeType<Speak>("Speak");
	factory.registerNodeType<Find>("Find");
	factory.registerNodeType<Pick>("Pick");
	factory.registerNodeType<Place>("Place");
	factory.registerNodeType<Failure>("Failure");
	factory.registerNodeType<MoveCloser>("MoveCloser");

	factory.registerSimpleCondition("ThingClose", std::bind(ThingClose));

	auto tree = factory.createTreeFromFile(argv[1]);

	tree.tickWhileRunning();

	return 0;
}