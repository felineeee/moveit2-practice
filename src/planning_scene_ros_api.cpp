#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>

class PlanningSceneRosApi : public rclcpp::Node
{
public:
  PlanningSceneRosApi() : Node("planning_scene_ros_api_tutorial")
  {
    // 1. Setup Visualization
    visual_tools_ = std::make_unique<rviz_visual_tools::RvizVisualTools>(
        "panda_link0", "planning_scene_ros_api_tutorial", shared_from_this());
    visual_tools_->loadRemoteControl();
    visual_tools_->deleteAllMarkers();

    // 2. Setup ROS Interfaces
    // Publisher for asynchronous updates (fire and forget)
    planning_scene_diff_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

    // Client for synchronous updates (wait for confirmation)
    apply_scene_client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");

    // Wait for subscribers/service
    while (planning_scene_diff_pub_->get_subscription_count() < 1)
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for planning scene subscribers...");
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
  }

  void runTutorial()
  {
    // Define a box object for the demo
    moveit_msgs::msg::AttachedCollisionObject box = createBox();

    // --- STEP 1: Add object to world ---
    visual_tools_->prompt("Next: Add object to world");
    publishSceneDiff(box.object);

    // --- STEP 2: Synchronous update via Service ---
    visual_tools_->prompt("Next: Add object via Service (Synchronous)");
    applySceneViaService(box.object);

    // --- STEP 3: Attach object to robot ---
    visual_tools_->prompt("Next: Attach object to robot hand");
    attachObject(box);

    // --- STEP 4: Detach object ---
    visual_tools_->prompt("Next: Detach object and return to world");
    detachObject(box);

    // --- STEP 5: Remove from world ---
    visual_tools_->prompt("Next: Remove object from world entirely");
    removeObjectFromWorld(box.object.id);

    RCLCPP_INFO(this->get_logger(), "Tutorial Complete!");
  }

private:
  moveit_msgs::msg::AttachedCollisionObject createBox()
  {
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = "panda_hand";
    attached_object.object.header.frame_id = "panda_hand";
    attached_object.object.id = "box";

    geometry_msgs::msg::Pose pose;
    pose.position.z = 0.11;
    pose.orientation.w = 1.0;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = { 0.075, 0.075, 0.075 };

    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);
    attached_object.object.operation = attached_object.object.ADD;

    // Allow the box to touch the hand without triggering collision
    attached_object.touch_links = { "panda_hand", "panda_leftfinger", "panda_rightfinger" };

    return attached_object;
  }

  void publishSceneDiff(const moveit_msgs::msg::CollisionObject& obj)
  {
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.world.collision_objects.push_back(obj);
    planning_scene_diff_pub_->publish(scene);
  }

  void applySceneViaService(const moveit_msgs::msg::CollisionObject& obj)
  {
    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    request->scene.is_diff = true;
    request->scene.world.collision_objects.push_back(obj);

    if (!apply_scene_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "Service not available");
      return;
    }

    auto result = apply_scene_client_->async_send_request(request);
    // In a real app, use callbacks; here we wait for tutorial flow
    if (rclcpp::spin_until_future_complete(shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Service call successful");
    }
  }

  void attachObject(const moveit_msgs::msg::AttachedCollisionObject& obj)
  {
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;

    // To attach: 1. Remove from world, 2. Add to RobotState
    moveit_msgs::msg::CollisionObject remove_cmd;
    remove_cmd.id = obj.object.id;
    remove_cmd.operation = remove_cmd.REMOVE;

    scene.world.collision_objects.push_back(remove_cmd);
    scene.robot_state.attached_collision_objects.push_back(obj);
    scene.robot_state.is_diff = true;

    planning_scene_diff_pub_->publish(scene);
  }

  void detachObject(const moveit_msgs::msg::AttachedCollisionObject& obj)
  {
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;

    // To detach: 1. Remove from RobotState, 2. Add back to world
    moveit_msgs::msg::AttachedCollisionObject detach_cmd = obj;
    detach_cmd.object.operation = detach_cmd.object.REMOVE;

    scene.robot_state.attached_collision_objects.push_back(detach_cmd);
    scene.robot_state.is_diff = true;
    scene.world.collision_objects.push_back(obj.object);

    planning_scene_diff_pub_->publish(scene);
  }

  void removeObjectFromWorld(const std::string& id)
  {
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    moveit_msgs::msg::CollisionObject remove_cmd;
    remove_cmd.id = id;
    remove_cmd.operation = remove_cmd.REMOVE;
    scene.world.collision_objects.push_back(remove_cmd);
    planning_scene_diff_pub_->publish(scene);
  }

  // Members
  std::unique_ptr<rviz_visual_tools::RvizVisualTools> visual_tools_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_pub_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanningSceneRosApi>();

  // Tutorial logic in a separate thread to keep the executor spinning
  std::thread worker([node]() { node->runTutorial(); });

  rclcpp::spin(node);
  worker.join();
  rclcpp::shutdown();
  return 0;
}