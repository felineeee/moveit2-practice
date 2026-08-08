#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <rclcpp/rclcpp.hpp>

class PlanningSceneTutorial : public rclcpp::Node {
public:
  PlanningSceneTutorial() : Node("planning_scene_tutorial") {
    // Initialize the RobotModelLoader
    // Using "robot_description" which is standard for MoveIt 2
    robot_model_loader_ =
        std::make_shared<robot_model_loader::RobotModelLoader>(
            shared_from_this(), "robot_description");

    kinematic_model_ = robot_model_loader_->getModel();

    if (!kinematic_model_) {
      RCLCPP_ERROR(this->get_logger(), "Could not load robot model");
      return;
    }

    // Initialize the Planning Scene
    planning_scene_ =
        std::make_unique<planning_scene::PlanningScene>(kinematic_model_);
  }

  /// @brief Runs the sequence of MoveIt tutorials
  void runTutorial() {
    performCollisionChecking();
    performConstraintChecking();
    performFeasibilityChecking();
  }

private:
  // --- Private Methods ---

  void performCollisionChecking() {
    RCLCPP_INFO(this->get_logger(), "--- Starting Collision Checking ---");

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    // 1. Basic Self Collision Check
    planning_scene_->checkSelfCollision(collision_request, collision_result);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Test 1: Current state "
                           << (collision_result.collision ? "in" : "not in")
                           << " self collision");

    // 2. Random State Collision Check
    moveit::core::RobotState &current_state =
        planning_scene_->getCurrentStateNonConst();
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene_->checkSelfCollision(collision_request, collision_result);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Test 2: Random state "
                           << (collision_result.collision ? "in" : "not in")
                           << " self collision");

    // 3. Allowed Collision Matrix (ACM) Modification
    // Manually force a collision state for demonstration
    std::vector<double> joint_values = {0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0};
    const moveit::core::JointModelGroup *joint_model_group =
        current_state.getJointModelGroup("panda_arm");
    current_state.setJointGroupPositions(joint_model_group, joint_values);

    collision_request.contacts = true;
    collision_request.max_contacts = 1000;
    collision_result.clear();
    planning_scene_->checkSelfCollision(collision_request, collision_result);

    collision_detection::AllowedCollisionMatrix acm =
        planning_scene_->getAllowedCollisionMatrix();
    for (const auto &contact : collision_result.contacts) {
      acm.setEntry(contact.first.first, contact.first.second, true);
    }

    collision_result.clear();
    planning_scene_->checkSelfCollision(collision_request, collision_result,
                                        current_state, acm);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Test 6: State with modified ACM is "
                           << (collision_result.collision ? "in" : "not in")
                           << " self collision");
  }

  void performConstraintChecking() {
    RCLCPP_INFO(this->get_logger(), "--- Starting Constraint Checking ---");

    moveit::core::RobotState &current_state =
        planning_scene_->getCurrentStateNonConst();
    const moveit::core::JointModelGroup *joint_model_group =
        current_state.getJointModelGroup("panda_arm");
    std::string end_effector_name =
        joint_model_group->getLinkModelNames().back();

    geometry_msgs::msg::PoseStamped desired_pose;
    desired_pose.pose.orientation.w = 1.0;
    desired_pose.pose.position.x = 0.3;
    desired_pose.pose.position.y = -0.185;
    desired_pose.pose.position.z = 0.5;
    desired_pose.header.frame_id = "panda_link0";

    moveit_msgs::msg::Constraints goal_constraint =
        kinematic_constraints::constructGoalConstraints(end_effector_name,
                                                        desired_pose);

    kinematic_constraints::KinematicConstraintSet k_constraint_set(
        kinematic_model_);
    k_constraint_set.add(goal_constraint, planning_scene_->getTransforms());

    bool constrained =
        planning_scene_->isStateConstrained(current_state, k_constraint_set);
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Test 9: State is "
            << (constrained ? "constrained" : "not constrained"));
  }

  void performFeasibilityChecking() {
    RCLCPP_INFO(this->get_logger(), "--- Starting Feasibility Checking ---");

    // Set user-defined predicate
    planning_scene_->setStateFeasibilityPredicate(
        [](const moveit::core::RobotState &robot_state, bool /*verbose*/) {
          const double *joint_values =
              robot_state.getJointPositions("panda_joint1");
          return (joint_values[0] > 0.0);
        });

    moveit::core::RobotState &current_state =
        planning_scene_->getCurrentStateNonConst();
    bool feasible = planning_scene_->isStateFeasible(current_state);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Test 11: State feasibility: "
                           << (feasible ? "feasible" : "not feasible"));
  }

  // --- Member Variables ---
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;
  std::unique_ptr<planning_scene::PlanningScene> planning_scene_;
};

// --- Main Function ---

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<PlanningSceneTutorial>();

  // Use a thread to spin so we can execute our tutorial logic on the main
  // thread
  std::thread spin_thread([node]() { rclcpp::spin(node); });

  // Run the logic
  node->runTutorial();

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}