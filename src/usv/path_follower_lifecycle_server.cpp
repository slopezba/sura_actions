#include "sura_actions/usv/path_follower_lifecycle_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sura_actions::usv::PathFollowerLifecycleServer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  executor.remove_node(node->get_node_base_interface());
  node.reset();
  rclcpp::shutdown();
  return 0;
}
