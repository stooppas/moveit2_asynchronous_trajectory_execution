#include "paper_benchmarks/benchmark_asynchronous.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("move_group");

  rclcpp::spin(node);
  rclcpp::shutdown();
}