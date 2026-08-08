#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/display_trajectory.hpp>

/**
 * @brief Professional implementation of the Motion Planning API.
 * Uses pluginlib to load planners and handles low-level PlanningContexts.
 */
class MotionPlanningApiTutorial {
public:
    MotionPlanningApiTutorial(const rclcpp::Node::SharedPtr& node, const std::string& planning_group);

    void planToPoseGoal();
    void planToJointGoal();
    void planWithPathConstraints();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    std::string planning_group_;

    // MoveIt Core Components
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    
    planning_scene::PlanningScenePtr planning_scene_;
    const moveit::core::JointModelGroup* joint_model_group_;

    // Planner Plugin Components
    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader_;
    planning_interface::PlannerManagerPtr planner_instance_;

    // Visualization
    moveit_visual_tools::MoveItVisualTools visual_tools_;
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher_;

    void initializePlanner();
    void visualizeAndLog(planning_interface::MotionPlanResponse& res, const std::string& label);
};

MotionPlanningApiTutorial::MotionPlanningApiTutorial(const rclcpp::Node::SharedPtr& node, const std::string& planning_group)
    : node_(node), logger_(node->get_logger()), planning_group_(planning_group),
      visual_tools_(node, "panda_link0", "motion_planning_api", moveit::core::RobotModelPtr()) {
    
    // 1. Setup Robot Model and State
    robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
    robot_model_ = robot_model_loader.getModel();
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    joint_model_group_ = robot_state_->getJointModelGroup(planning_group_);
    
    // 2. Setup Planning Scene
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    planning_scene_->getCurrentStateNonConst().setToDefaultValues(joint_model_group_, "ready");

    // 3. Setup Visualization
    display_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
    visual_tools_.loadRemoteControl();

    initializePlanner();
}

void MotionPlanningApiTutorial::initializePlanner() {
    std::vector<std::string> planner_plugin_names;
    if (!node_->get_parameter("ompl.planning_plugins", planner_plugin_names)) {
        RCLCPP_FATAL(logger_, "Could not find planner plugin names");
    }

    planner_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
        "moveit_core", "planning_interface::PlannerManager");

    const auto& planner_name = planner_plugin_names.at(0);
    planner_instance_.reset(planner_plugin_loader_->createUnmanagedInstance(planner_name));
    planner_instance_->initialize(robot_model_, node_, node_->get_namespace());
    
    RCLCPP_INFO(logger_, "Using planning interface '%s'", planner_instance_->getDescription().c_str());
}

void MotionPlanningApiTutorial::planToPoseGoal() {
    visual_tools_.prompt("Press 'next' for Pose Goal plan");

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "panda_link0";
    pose.pose.position.x = 0.3;
    pose.pose.position.y = 0.4;
    pose.pose.position.z = 0.75;
    pose.pose.orientation.w = 1.0;

    req.group_name = planning_group_;
    req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints("panda_link8", pose));

    auto context = planner_instance_->getPlanningContext(planning_scene_, req, res.error_code);
    context->solve(res);
    visualizeAndLog(res, "Pose Goal");
}

void MotionPlanningApiTutorial::planToJointGoal() {
    visual_tools_.prompt("Press 'next' for Joint Space Goal plan");

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name = planning_group_;

    moveit::core::RobotState goal_state(robot_model_);
    std::vector<double> joint_values = { -1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0 };
    goal_state.setJointGroupPositions(joint_model_group_, joint_values);
    
    req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group_));

    auto context = planner_instance_->getPlanningContext(planning_scene_, req, res.error_code);
    context->solve(res);
    visualizeAndLog(res, "Joint Goal");
}

void MotionPlanningApiTutorial::planWithPathConstraints() {
    visual_tools_.prompt("Press 'next' for Orientation Constrained plan");

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name = planning_group_;

    // Target Pose
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "panda_link0";
    pose.pose.position.x = 0.32;
    pose.pose.position.y = -0.25;
    pose.pose.position.z = 0.65;
    pose.pose.orientation.w = 1.0;
    req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints("panda_link8", pose));

    // Orientation Constraint: Keep end-effector level (quaternion identity)
    geometry_msgs::msg::QuaternionStamped quaternion;
    quaternion.header.frame_id = "panda_link0";
    quaternion.quaternion.w = 1.0;
    req.path_constraints = kinematic_constraints::constructGoalConstraints("panda_link8", quaternion);

    // Increase workspace bounds for complex constrained planning
    req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = req.workspace_parameters.min_corner.z = -2.0;
    req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.z = 2.0;

    auto context = planner_instance_->getPlanningContext(planning_scene_, req, res.error_code);
    context->solve(res);
    visualizeAndLog(res, "Constrained Goal");
}

void MotionPlanningApiTutorial::visualizeAndLog(planning_interface::MotionPlanResponse& res, const std::string& label) {
    if (res.error_code.val != res.error_code.SUCCESS) {
        RCLCPP_ERROR(logger_, "Planning failed for: %s", label.c_str());
        return;
    }

    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    moveit_msgs::msg::MotionPlanResponse response_msg;
    res.getMessage(response_msg);

    display_trajectory.trajectory_start = response_msg.trajectory_start;
    display_trajectory.trajectory.push_back(response_msg.trajectory);
    
    visual_tools_.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group_);
    visual_tools_.trigger();
    display_publisher_->publish(display_trajectory);

    // Update the planning scene with the final state of the trajectory
    robot_state_->setJointGroupPositions(joint_model_group_, response_msg.trajectory.joint_trajectory.points.back().positions);
    planning_scene_->setCurrentState(*robot_state_);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("motion_planning_api_node", node_options);

    // Standard ROS 2 executor pattern
    std::thread worker([node]() {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    });

    MotionPlanningApiTutorial tutorial(node, "panda_arm");
    
    tutorial.planToPoseGoal();
    tutorial.planToJointGoal();
    tutorial.planWithPathConstraints();

    rclcpp::shutdown();
    worker.join();
    return 0;
}