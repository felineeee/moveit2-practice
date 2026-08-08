#include <rclcpp/rclcpp.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>


class DisplayRobot{
public:
    DisplayRobot(const rclcpp::Node::SharedPtr& node);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;

    // robot model and state
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;

    // rviz
    rviz_visual_tools::RvizVisualTools visual_tools_;

    // clcpp
    // Robot state
    rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr state_pub_;
    // Planning scene
    /** choose between SYNC and ASYNC */
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_client_;

    // Object definition
    moveit_msgs::msg::AttachedCollisionObject object_;
    
    // Robot Model Loader
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
};

DisplayRobot::DisplayRobot(const rclcpp::Node::SharedPtr& node)
    : node_(node), 
      logger_(node->get_logger()),
      visual_tools_("panda_link0", "planning_scene_ros_api", node) {

    // robot model
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description");
    robot_model_ = robot_model_loader_->getModel();

    if (!robot_model_) {
        RCLCPP_ERROR(logger_, "Could not load robot model!");
        return;
    }

    // robot state
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();
    planning_scene_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

    state_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 10);

    RCLCPP_INFO(logger_, "Class initialized successfully.");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node_options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("collision_checker_node", node_options);
    
    auto collision_checker = std::make_shared<DisplayRobot>(node);
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    rclcpp::shutdown();
    return 0;
}