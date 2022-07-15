#include "paper_benchmarks/benchmark_baseline.hpp"
#include <moveit_msgs/srv/grasp_planning.h>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  node = std::make_shared<rclcpp::Node>("benchmark_baseline");

  scene = std::make_shared<Scene>(node);

  new std::thread(main_thread);

  rclcpp::spin(node);
  rclcpp::shutdown();
}

void main_thread()
{

}