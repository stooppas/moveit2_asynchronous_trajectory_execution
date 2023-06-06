#include "paper_benchmarks/benchmark_baseline.hpp"

using namespace std::chrono_literals;

int number_of_test_cases = 5;

static struct runner{
  int counter = 1;
  std::mutex mtx;

  void increment(){
    std::lock_guard<std::mutex> lock(mtx);
    counter++;
  }

  int check(){
    std::lock_guard<std::mutex> lock(mtx);
    return counter;
  }
} runner1;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("benchmark_baseline");

  node->declare_parameter("cubesToPick", 5);

  number_of_test_cases = node->get_parameter("cubesToPick").as_int();

  pnp = std::make_shared<primitive_pick_and_place>(node, "panda_1");

  publisher_ = node->create_publisher<std_msgs::msg::String>("spawnNewCube", 10);

  new std::thread(update_planning_scene);

  new std::thread(main_thread);

  rclcpp::spin(node);
  rclcpp::shutdown();
}

void update_planning_scene()
{

  while (true)
  {

    objMap = pnp->getCollisionObjects();
    colors = pnp->getCollisionObjectColors();

    for (auto &pair : objMap)
    {
      auto it = std::find(all_objects.begin(), all_objects.end(), pair.second.id);

      // not found in the object list
      if (it == all_objects.end())
      {
        all_objects.push_back(pair.second.id);

        CollisionPlanningObject new_object(pair.second, 0, 0);
        objs.push(new_object);

        RCLCPP_INFO(LOGGER, "New object detected. id: %s", pair.second.id.c_str());
      }
    }

    std::this_thread::sleep_for(1.0s);
    update_scene_called_once = true;
  }
}

void main_thread()
{
  srand(time(0));
  rclcpp::Rate r(1);
  bool success = false;

  pnp->home();

  pnp->open_gripper();

  while (!update_scene_called_once)
  {
    std::this_thread::sleep_for(1.0s);
  }

  RCLCPP_INFO(LOGGER, "Size: %li", objs.size());

  geometry_msgs::msg::Pose pose;

  tray_helper blue_tray(6, 3, 0.11, -0.925, 0.06, 0.1, true);
  tray_helper red_tray(6, 3, -0.425, -0.925, 0.06, 0.1, true);
  tray_helper *active_tray;

  RCLCPP_INFO(LOGGER, "[checkpoint] Starting the baseline processing with %i cubes", number_of_test_cases);

  int pregrasp_executing_retries = 0;
  int grasp_executing_retries = 0;

  while (!objs.empty())
  {
    auto obj = objs.pop("", "random").collisionObject;
    RCLCPP_INFO(LOGGER, "Object: %s", obj.id.c_str());

    // Check if the object is a box
    if (obj.id.rfind("box", 0) != 0)
    {
      RCLCPP_INFO(LOGGER, "Skipping");
      continue;
    }

    if (colors[obj.id].color.r == 1 && colors[obj.id].color.g == 0 && colors[obj.id].color.b == 0)
      active_tray = &red_tray;
    else if (colors[obj.id].color.r == 0 && colors[obj.id].color.g == 0 && colors[obj.id].color.b == 1)
      active_tray = &blue_tray;
    else
      continue;

    // Pre Grasp
    pose.position.x = obj.pose.position.x;
    pose.position.y = obj.pose.position.y;
    pose.position.z = obj.pose.position.z + 0.25;

    pose.orientation.x = obj.pose.orientation.w;
    pose.orientation.y = obj.pose.orientation.z;
    pose.orientation.z = 0;
    pose.orientation.w = 0;

    tf2::Quaternion q_orig, q_rot, q_new;

    tf2::convert(pose.orientation, q_orig);

    tf2::Matrix3x3 m(q_orig);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    int attempts = 1;
    float offset = 0;

    bool failed = false;

    while (!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
    {
      RCLCPP_INFO(LOGGER, "Try again pre grasp failed");
      if(!pnp->is_execution_successful())
      {
        RCLCPP_ERROR(LOGGER, "Pre grasp execution failed");
        if(pregrasp_executing_retries >= 2)
        {
          pregrasp_executing_retries = 0;
          grasp_executing_retries = 0;
          failed = true;
          break;
        }
        else
        {
          pregrasp_executing_retries++;
          RCLCPP_ERROR(LOGGER, "Retrying pregrasp execution");
        }
      }
    }

    if (failed){
      continue;
    }

    pnp->set_default();

    // Grasp
    pose.position.z = obj.pose.position.z + 0.1;

    while (!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
    {
      RCLCPP_INFO(LOGGER, "Try again grasp failed");
      if(!pnp->is_execution_successful())
      {
        RCLCPP_ERROR(LOGGER, "Grasp execution failed");
        if(grasp_executing_retries >= 2)
        {
          pregrasp_executing_retries = 0;
          grasp_executing_retries = 0;
          failed = true;
          break;
        }
        else
        {
          grasp_executing_retries++;
          RCLCPP_ERROR(LOGGER, "Retrying grasp execution");
        }
      }
    }

    if (failed){
      continue;
    }

    pnp->grasp_object(obj);

    // Pre Move
    pose.position.z = obj.pose.position.z + 0.25;

    while (!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
    {
      RCLCPP_INFO(LOGGER, "Retrying");
    }

    // Move
    pose.position.x = active_tray->get_x();
    pose.position.y = active_tray->get_y();
    pose.position.z = 1.28 + active_tray->z * 0.05;

    pose.orientation.x = 1;
    pose.orientation.y = 0;

    while (!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
    {
      RCLCPP_INFO(LOGGER, "Retrying");
    }
    // Put down
    pose.position.z = 1.141 + active_tray->z * 0.05;

    while (!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
    {
      RCLCPP_INFO(LOGGER, "Retrying");
    }
    pnp->release_object(obj);
    // Post Move
    pose.position.z = 1.28 + active_tray->z * 0.05;

    while (!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
    {
      RCLCPP_INFO(LOGGER, "Retrying");
    }
    active_tray->next();
    success = true;
    r.sleep();

    if(runner1.check() > number_of_test_cases){
      RCLCPP_INFO(LOGGER, "[terminate]");
    }else{
      runner1.increment();
      RCLCPP_INFO(LOGGER, "[checkpoint] Robot successful placing. Request to spawn a new cube ");
    }

    // spawn two new cubes
    auto message = std_msgs::msg::String();
    publisher_->publish(message);
  }

  RCLCPP_INFO(LOGGER, "Finished");
}