#include "paper_benchmarks/scene.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

using std::placeholders::_1;

class SceneCreator : public rclcpp::Node
{
  public:
    SceneCreator(): Node("create_scene")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>("spawnNewCube", 10, 
                      std::bind(&SceneCreator::addRandomObject, this, _1));
      timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&SceneCreator::createRandomScene, this));
    }

  private:
    void addRandomObject(const std_msgs::msg::String::SharedPtr msg) const
    {
      scene->add_objects_to_scene(1);
    }

    // need to be called only once
    void createRandomScene(){
      if(!_executed){
        node = shared_from_this();
        scene = std::make_shared<Scene>(node);
        scene->create_random_scene();
        _executed = true;
      }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<Scene> scene;
    std::shared_ptr<rclcpp::Node> node;
    bool _executed = false;
};



int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SceneCreator>());
  rclcpp::shutdown();
  return 0;
}
