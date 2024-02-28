#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include "example_bt_nodes.hpp"

#include <chrono>
using namespace std::chrono_literals;

using namespace BT;


int main(int argc, char **argv)
{
  BehaviorTreeFactory factory;
  factory.registerNodeType<MoveBaseAction>("MoveBase");
  factory.registerNodeType<Speak>("Speak");

  auto tree = factory.createTreeFromFile(argv[1]);
 
  // Here, instead of tree.tickWhileRunning(),
  // we prefer our own loop.
  std::cout << "--- ticking\n";
  auto status = tree.tickOnce();
  std::cout << "--- status: " << toStr(status) << "\n\n";

  while(status == NodeStatus::RUNNING) {
  	// Put in a sleep to avoid busy waiting.  Usually this will be smaller than 100ms, but we're setting it high
  	// here to have a reasonable amount of output to the screen.  We should only use tree.sleep() since it allows
  	// for preemption.
    tree.sleep(100ms);

    std::cout << "--- ticking\n";
    status = tree.tickOnce();
    std::cout << "--- status: " << toStr(status) << "\n\n";
  }

  return 0;
}