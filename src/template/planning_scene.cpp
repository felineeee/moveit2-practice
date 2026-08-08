#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/kinematic_constraints/utils.hpp>

/**
 * @brief Professional implementation of the Planning Scene Tutorial.
 * This class demonstrates how to manage world state, collision detection, and constraints.
 */
class PlanningSceneTutorial
{
public:
  PlanningSceneTutorial(const rclcpp::Node::SharedPtr& node);

  // Collision Detection Methods
  void checkSelfCollision();
  void checkGroupCollision(const std::string& group_name);
  void getContactInformation(const std::string& group_name);
  void modifyAllowedCollisionMatrix();
  void fullCollisionChecking();

  // Constraint Checking Methods
  void checkKinematicConstraints(const std::string& group_name);
  void checkUserDefinedConstraints();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;

  moveit::core::RobotModelPtr kinematic_model_;
  planning_scene::PlanningScenePtr planning_scene_;

  // Static callback for feasibility testing
  static bool stateFeasibilityPredicate(const moveit::core::RobotState& robot_state, bool verbose);
};

PlanningSceneTutorial::PlanningSceneTutorial(const rclcpp::Node::SharedPtr& node)
  : node_(node), logger_(node->get_logger())
{
  // Setup Robot Model Loader
  robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
  kinematic_model_ = robot_model_loader.getModel();

  // Instantiate Planning Scene
  planning_scene_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model_);
}

void PlanningSceneTutorial::checkSelfCollision()
{
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  planning_scene_->checkSelfCollision(collision_request, collision_result);
  RCLCPP_INFO_STREAM(logger_, "Test 1: Current state is " << (collision_result.collision ? "in" : "not in")
                                                          << " self-collision");

  // Randomize and check again
  moveit::core::RobotState& current_state = planning_scene_->getCurrentStateNonConst();
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene_->checkSelfCollision(collision_request, collision_result);
  RCLCPP_INFO_STREAM(logger_,
                     "Test 2: Random state is " << (collision_result.collision ? "in" : "not in") << " self-collision");
}

void PlanningSceneTutorial::checkGroupCollision(const std::string& group_name)
{
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.group_name = group_name;

  planning_scene_->getCurrentStateNonConst().setToRandomPositions();
  planning_scene_->checkSelfCollision(collision_request, collision_result);
  RCLCPP_INFO_STREAM(logger_, "Test 3: Group '" << group_name << "' is "
                                                << (collision_result.collision ? "in" : "not in") << " self-collision");
}

void PlanningSceneTutorial::getContactInformation(const std::string& group_name)
{
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;

  // Manually set a colliding state for Panda
  std::vector<double> joint_values = { 0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0 };
  moveit::core::RobotState& current_state = planning_scene_->getCurrentStateNonConst();
  const moveit::core::JointModelGroup* jmg = current_state.getJointModelGroup(group_name);
  current_state.setJointGroupPositions(jmg, joint_values);

  planning_scene_->checkSelfCollision(collision_request, collision_result);

  for (const auto& contact : collision_result.contacts)
  {
    RCLCPP_INFO(logger_, "Contact: %s and %s", contact.first.first.c_str(), contact.first.second.c_str());
  }
}

void PlanningSceneTutorial::modifyAllowedCollisionMatrix()
{
  collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;

  // Example: Tell ACM to ignore all collisions between all links (For demonstration)
  acm.setEntry("panda_link4", "panda_link2", true);

  planning_scene_->checkSelfCollision(req, res, planning_scene_->getCurrentState(), acm);
  RCLCPP_INFO_STREAM(logger_, "Test 6 (with ACM): State is " << (res.collision ? "in" : "not in") << " collision");
}

void PlanningSceneTutorial::checkKinematicConstraints(const std::string& group_name)
{
  const moveit::core::JointModelGroup* jmg = kinematic_model_->getJointModelGroup(group_name);
  std::string ee_name = jmg->getLinkModelNames().back();

  geometry_msgs::msg::PoseStamped desired_pose;
  desired_pose.pose.orientation.w = 1.0;
  desired_pose.pose.position.x = 0.3;
  desired_pose.header.frame_id = "panda_link0";

  moveit_msgs::msg::Constraints goal_constraint =
      kinematic_constraints::constructGoalConstraints(ee_name, desired_pose);

  moveit::core::RobotState random_state(kinematic_model_);
  random_state.setToRandomPositions();

  bool constrained = planning_scene_->isStateConstrained(random_state, goal_constraint);
  RCLCPP_INFO_STREAM(logger_, "Test 8: State satisfies constraint: " << (constrained ? "YES" : "NO"));
}

bool PlanningSceneTutorial::stateFeasibilityPredicate(const moveit::core::RobotState& robot_state, bool /*verbose*/)
{
  const double* joint_values = robot_state.getJointPositions("panda_joint1");
  return (joint_values[0] > 0.0);  // Only allow positive joint angles for joint 1
}

void PlanningSceneTutorial::checkUserDefinedConstraints()
{
  planning_scene_->setStateFeasibilityPredicate(stateFeasibilityPredicate);
  bool feasible = planning_scene_->isStateFeasible(planning_scene_->getCurrentState());
  RCLCPP_INFO_STREAM(logger_, "Test 11: State is feasible: " << (feasible ? "YES" : "NO"));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("planning_scene_tutorial_node", node_options);

  // Spin in a separate thread for MoveIt background tasks
  std::thread worker([node]() {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  });

  PlanningSceneTutorial tutorial(node);

  tutorial.checkSelfCollision();
  tutorial.checkGroupCollision("hand");
  tutorial.getContactInformation("panda_arm");
  tutorial.modifyAllowedCollisionMatrix();
  tutorial.checkKinematicConstraints("panda_arm");
  tutorial.checkUserDefinedConstraints();

  rclcpp::shutdown();
  worker.join();
  return 0;
}