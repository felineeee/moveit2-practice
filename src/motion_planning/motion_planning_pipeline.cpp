#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/display_trajectory.hpp>

/**
 * @brief Professional implementation of the Motion Planning Pipeline.
 * Demonstrates the use of PlanningSceneMonitor and Request Adapters.
 */
class MotionPlanningPipelineTutorial {
public:
    MotionPlanningPipelineTutorial(const rclcpp::Node::SharedPtr& node, const std::string& group_name);

    void planToPoseGoal();
    void planToJointGoal();
    void planWithAdapterFix();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    std::string planning_group_;

    // Core Components
    moveit::core::RobotModelPtr robot_model_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    planning_pipeline::PlanningPipelinePtr planning_pipeline_;

    // Visualization
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher_;

    void setupSceneMonitor();
    void visualizeAndLog(const planning_interface::MotionPlanResponse& res, const moveit::core::JointModelGroup* jmg);
};

MotionPlanningPipelineTutorial::MotionPlanningPipelineTutorial(const rclcpp::Node::SharedPtr& node, const std::string& group_name)
    : node_(node), logger_(node->get_logger()), planning_group_(group_name) {
    
    // 1. Load Robot Model
    auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description");
    robot_model_ = robot_model_loader->getModel();

    // 2. Setup Planning Scene Monitor (PSM)
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, robot_model_loader);
    setupSceneMonitor();

    // 3. Setup Planning Pipeline
    // This loads the planner plugin (e.g., OMPL) and adapters from parameters
    planning_pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model_, node_, "ompl");

    // 4. Visualization
    display_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
    
    // Initialize Visual Tools after robot_model_ is loaded
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node, "panda_link0", "pipeline_tutorial", robot_model_);
    visual_tools_->loadRemoteControl();
}

void MotionPlanningPipelineTutorial::setupSceneMonitor() {
    psm_->startSceneMonitor();
    psm_->startWorldGeometryMonitor();
    psm_->startStateMonitor();
}

void MotionPlanningPipelineTutorial::planToPoseGoal() {
    visual_tools_->prompt("Press 'next' for Pipeline Pose Goal");

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.pipeline_id = "ompl";
    req.group_name = planning_group_;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "panda_link0";
    pose.pose.position.x = 0.3;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.75;
    pose.pose.orientation.w = 1.0;

    std::vector<double> tolerance(3, 0.01);
    req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance, tolerance));

    // Lock the scene for planning
    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
        if (!planning_pipeline_->generatePlan(lscene, req, res)) {
            RCLCPP_ERROR(logger_, "Could not generate plan");
        }
    }

    auto jmg = robot_model_->getJointModelGroup(planning_group_);
    visualizeAndLog(res, jmg);
}

void MotionPlanningPipelineTutorial::planToJointGoal() {
    visual_tools_->prompt("Press 'next' for Pipeline Joint Goal");

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name = planning_group_;

    // Set Start State from current PSM state
    moveit::core::RobotStatePtr start_state;
    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
        start_state = std::make_shared<moveit::core::RobotState>(lscene->getCurrentState());
    }
    
    moveit::core::robotStateToRobotStateMsg(*start_state, req.start_state);

    // Define Goal State
    moveit::core::RobotState goal_state(*start_state);
    std::vector<double> joint_values = { -1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0 };
    goal_state.setJointGroupPositions(planning_group_, joint_values);
    req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state, goal_state.getJointModelGroup(planning_group_)));

    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
        if (!planning_pipeline_->generatePlan(lscene, req, res)) {
            RCLCPP_ERROR(logger_, "Could not generate plan");
        }
    }

    visualizeAndLog(res, goal_state.getJointModelGroup(planning_group_));
}

void MotionPlanningPipelineTutorial::planWithAdapterFix() {
    visual_tools_->prompt("Press 'next' to see Adapter fixing an invalid joint state");

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name = planning_group_;

    // Intentionally set a joint slightly outside its bounds
    moveit::core::RobotStatePtr invalid_state;
    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
        invalid_state = std::make_shared<moveit::core::RobotState>(lscene->getCurrentState());
    }
    
    const moveit::core::JointModel* jm3 = invalid_state->getJointModel("panda_joint3");
    double out_of_bounds = jm3->getVariableBounds()[0].min_position_ - 0.01;
    invalid_state->setJointPositions(jm3, &out_of_bounds);
    moveit::core::robotStateToRobotStateMsg(*invalid_state, req.start_state);

    // Reuse the pose goal from earlier
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "panda_link0";
    pose.pose.position.x = 0.3;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.75;
    pose.pose.orientation.w = 1.0;
    req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints("panda_link8", pose));

    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
        // The 'FixStartStateBounds' adapter will handle the out-of-bounds joint automatically
        if (!planning_pipeline_->generatePlan(lscene, req, res)) {
            RCLCPP_ERROR(logger_, "Could not generate plan");
        }
    }

    visualizeAndLog(res, invalid_state->getJointModelGroup(planning_group_));
}

void MotionPlanningPipelineTutorial::visualizeAndLog(const planning_interface::MotionPlanResponse& res, const moveit::core::JointModelGroup* jmg) {
    if (res.error_code.val != res.error_code.SUCCESS) {
        RCLCPP_ERROR(logger_, "Pipeline planning failed!");
        return;
    }

    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    moveit_msgs::msg::MotionPlanResponse response_msg;
    res.getMessage(response_msg);

    display_trajectory.trajectory_start = response_msg.trajectory_start;
    display_trajectory.trajectory.push_back(response_msg.trajectory);
    
    display_publisher_->publish(display_trajectory);
    
    // Visualize the trajectory line using the RobotTrajectory object directly
    visual_tools_->publishTrajectoryLine(res.trajectory, jmg);
    visual_tools_->trigger();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("pipeline_node", node_options);

    std::thread worker([node]() {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    });

    MotionPlanningPipelineTutorial tutorial(node, "panda_arm");
    
    tutorial.planToPoseGoal();
    tutorial.planToJointGoal();
    tutorial.planWithAdapterFix();

    rclcpp::shutdown();
    worker.join();
    return 0;
}