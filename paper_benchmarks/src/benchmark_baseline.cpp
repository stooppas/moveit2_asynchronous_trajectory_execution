#include "paper_benchmarks/benchmark_baseline.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  node = std::make_shared<rclcpp::Node>("benchmark_baseline");

  scene = std::make_shared<Scene>(node);

  pnp = std::make_shared<primitive_pick_and_place>(node,"panda_1");

  new std::thread(main_thread);

  rclcpp::spin(node);
  rclcpp::shutdown();
}

void main_thread()
{ 
    rclcpp::Rate r(1);
    bool success = false;
    scene->create_random_scene();
    pnp->open_gripper();

    auto planning_interface = new moveit::planning_interface::PlanningSceneInterface();

    std::map<std::string, moveit_msgs::msg::CollisionObject> objs = planning_interface->getObjects();

    RCLCPP_INFO(LOGGER,"Size: %i",objs.size());

    for(auto obj : objs){
      RCLCPP_INFO(LOGGER,"Object: %s",obj.first.c_str());

      //Check if the object is a box
      if(obj.first.rfind("box",0) != 0) continue;
      //Set the current object as the active object
      pnp->set_active_object(obj.second);

      if(pnp->generate_pre_grasp_pose() && pnp->plan_and_execute()){
        if(pnp->generate_grasp_pose() && pnp->plan_and_execute()){
          pnp->grasp_object();
          if(pnp->generate_post_grasp_pose() && pnp->plan_and_execute()){
            if(pnp->generate_move_pose()&& pnp->plan_and_execute()){
              if(pnp->generate_place_pose()&& pnp->plan_and_execute()){
                pnp->release_object();
                if(pnp->generate_post_place_pose()&& pnp->plan_and_execute()){
                  success = true;
                }
              }
            }
          }
        }
      }
      r.sleep();
    }
    

}