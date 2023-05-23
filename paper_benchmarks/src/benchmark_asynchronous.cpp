#include "paper_benchmarks/benchmark_asynchronous.hpp"
#include <thread>
#include <iostream>
#include "std_msgs/msg/string.hpp"
#include "paper_benchmarks/cube_selector.hpp"

using namespace std::chrono_literals;

std::mutex mtx;

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
std::vector<std::string> all_objects;
std::vector<moveit_msgs::msg::CollisionObject> objs;

std::map<std::string, moveit_msgs::msg::CollisionObject> objMap;
std::map<std::string, moveit_msgs::msg::ObjectColor> colors;
bool update_scene_called_once = false;


void main_thread2();

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  node = std::make_shared<rclcpp::Node>("benchmark_asynchronous");


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Test codes 
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  pnp_1 = std::make_shared<primitive_pick_and_place>(node,"panda_1");

  



  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // End of Test codes 
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  //pnp_1 = std::make_shared<primitive_pick_and_place>(node,"panda_1");
  //pnp_2 = std::make_shared<primitive_pick_and_place>(node,"panda_2");

  
  //publisher_ = node->create_publisher<std_msgs::msg::String>("spawnNewCube", 10);


  //we need another thread to update the obj when new object spawns in the environment
  //new std::thread(update_planning_scene);

  new std::thread(main_thread2);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
}

void main_thread2(){
  objMap = pnp_1->getCollisionObjects();

  CubeContainer container;

  for (const auto& pair : objMap) {     
    container.addCubes(pair.second);
  }

  Point3D e(0,0,0); //simulate end-effector

  for (auto it = container.beginEuclidean(e); it != container.endEuclidean(e); ++it) {
    std::cout << "(" << (*it).pose.position.x << ", " << (*it).pose.position.y << ", " << (*it).pose.position.z << ")" << std::endl;
  }
}

void update_planning_scene(){

  while(true){

    objMap = pnp_1->getCollisionObjects();
    colors = pnp_1->getCollisionObjectColors();  

    for (const auto& pair : objMap) {
      auto it = std::find(all_objects.begin(), all_objects.end(), pair.second.id);
      
      // not found in the object list
      if(it == all_objects.end()){
        all_objects.push_back(pair.second.id);

        // dummy scope for mutex
        {
        std::lock_guard<std::mutex> lock(mtx);
        objs.push_back(pair.second);
        }

        RCLCPP_INFO(LOGGER,"New object detected. id: %s", pair.second.id.c_str());
      }
      
    }

    std::this_thread::sleep_for(1.0s);
    update_scene_called_once = true;
    
  }  

}

void main_thread()
{ 
  rclcpp::Rate r(5);

  pnp_1->home();
  pnp_2->home();
  

  while(!update_scene_called_once){
    std::this_thread::sleep_for(1.0s);
  }

  RCLCPP_INFO(LOGGER,"Size: %li",objs.size());

  int i = 0;

  RCLCPP_INFO(LOGGER,"[checkpoint]");

  while(!objs.empty()){
    moveit_msgs::msg::CollisionObject current_object;
    
    // start planning if atleast one of the arms are available
    if(!panda_1_busy || !panda_2_busy){

      {
        std::lock_guard<std::mutex> lock(mtx);
        current_object = objs.front();
        objs.erase(objs.begin());
        RCLCPP_INFO(LOGGER,"[checkpoint]");
        if(i==3){
          RCLCPP_INFO(LOGGER,"[terminate]");
        }else{
        i++;
        }
      }

      auto active_object = current_object;
      auto object_id = current_object.id;
      RCLCPP_INFO(LOGGER,"Object: %s",object_id.c_str());

      //Check if the object is a box
      if(object_id.rfind("box",0) != 0){
        continue;
      }

      bool panda_1_success = true;
      bool panda_2_success = true;

      // plan for if the arm one is not busy
      if(!panda_1_busy)
      {
        tray_helper* active_tray;
        if(colors[object_id].color.r == 1 && colors[object_id].color.g == 0 && colors[object_id].color.b == 0)active_tray = &red_tray_1;
        else if(colors[object_id].color.r == 0 && colors[object_id].color.g == 0 && colors[object_id].color.b == 1)active_tray = &blue_tray_1;
        else continue;

        new std::thread([&](){
          panda_1_busy = true;
          auto current_object_1 = std::move(current_object);
          bool panda_1_success = executeTrajectory(pnp_1, current_object_1,active_tray);
          
          if(!panda_1_success)
          {
            std::lock_guard<std::mutex> lock(mtx);
            objs.push_back(current_object_1);
          }else{
            RCLCPP_ERROR(LOGGER, "spawing new cube <------------------------------------>");
            auto message = std_msgs::msg::String();
            publisher_->publish(message);
          }
          panda_1_busy = false;
        });
        std::this_thread::sleep_for(0.2s);
        
      }
      
      // plan for if the arm one is not busy
      else if (!panda_2_busy)
      {
        tray_helper* active_tray;
        if(colors[object_id].color.r == 1 && colors[object_id].color.g == 0 && colors[object_id].color.b == 0)active_tray = &red_tray_2;
        else if(colors[object_id].color.r == 0 && colors[object_id].color.g == 0 && colors[object_id].color.b == 1)active_tray = &blue_tray_2;
        else continue;
        new std::thread([&](){
          panda_2_busy = true;
          auto current_object_2 = std::move(current_object);
          panda_2_success = executeTrajectory(pnp_2, current_object_2,active_tray);
          
          if(!panda_2_success)
          {
            std::lock_guard<std::mutex> lock(mtx);
            objs.push_back(current_object_2);
          }else{
            RCLCPP_ERROR(LOGGER, "spawing new cube <------------------------------------>");
            auto message = std_msgs::msg::String();
            publisher_->publish(message);
          }
          panda_2_busy = false;
        });
        std::this_thread::sleep_for(0.2s);
        
      }
    }
    r.sleep();
  }

  RCLCPP_INFO(LOGGER, "Execution completed" );

}

bool executeTrajectory(std::shared_ptr<primitive_pick_and_place> pnp,moveit_msgs::msg::CollisionObject& object,tray_helper* tray)
{ 
  RCLCPP_INFO(LOGGER,"Start execution of Object: %s",object.id.c_str());
  geometry_msgs::msg::Pose pose;
  pnp->open_gripper();

  std::this_thread::sleep_for(0.2s);

  //Pre Grasp
  pose.position.x = object.pose.position.x;
  pose.position.y = object.pose.position.y;
  pose.position.z = object.pose.position.z + 0.25;

  pose.orientation.x = object.pose.orientation.w;
  pose.orientation.y = object.pose.orientation.z;
  pose.orientation.z = 0;
  pose.orientation.w = 0;

  while(!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER,"Try again pre grasp failed");
    if(!pnp->is_plan_successful()){
        RCLCPP_ERROR(LOGGER,"Pre grasp Planner failed");
        return false;
    }
  }
  //Grasp
  pose.position.z = object.pose.position.z + 0.1;

  while(!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER,"Try again grasp failed");
    if(!pnp->is_plan_successful()){
        RCLCPP_ERROR(LOGGER,"Pre Grasp Planner failed");
        return false;
    }
  }
  pnp->grasp_object(object);
  RCLCPP_INFO(LOGGER,"Grasping object with ID %s",object.id.c_str());

  //Once grasped, no turning back! From now, ensure execution with while

  //Pre Move
  pose.position.z = object.pose.position.z + 0.25;

  while(!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER,"Try again pre move failed");
  }

  //Move
  pose.position.x = tray->get_x();
  pose.position.y = tray->get_y();
  pose.position.z = 1.28 + tray->z * 0.05;

  pose.orientation.x = 1;
  pose.orientation.y = 0;

  while(!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER,"Try again move failed");
  }
  // Put down
  pose.position.z = 1.141 + tray->z * 0.05;

  while(!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER,"Try again put down failed");
  }
  pnp->release_object(object);
  //Post Move
  pose.position.z = 1.28 + tray->z * 0.05; 

  while(!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER,"Try again post move failed");
  }
  tray->next();

  return true;
}