#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>

class MoveGroupApp {
public:
    MoveGroupApp(const rclcpp::Node::SharedPtr& node, const std::string& planning_group);

    // Planning Methods
    void showLogger();
    bool planToPose();
    bool planToJointSpace();
    void planCartesianPath();
    void planWithObstacle();
    bool executePlan();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
private:
    rclcpp::NodeOptions node_options;
    rclcpp::Node::SharedPtr node_;

    const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    moveit::core::RobotState default_state_;

    const moveit::core::JointModelGroup* joint_group_;
    moveit_visual_tools::MoveItVisualTools visual_tools_;
    
    moveit::planning_interface::MoveGroupInterface::Plan current_plan_;
};
MoveGroupApp::MoveGroupApp(const rclcpp::Node::SharedPtr& node, const std::string& planning_group)
    : node_(node),
      move_group_(node, planning_group),
      default_state_(*move_group_.getCurrentState()),
      visual_tools_(node, "panda_link0", "/rviz_visual_tools", move_group_.getRobotModel()) 
{
    // Raw pointers for performance
    joint_group_ = move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    default_state_ = *move_group_.getCurrentState();

    visual_tools_.deleteAllMarkers();
    visual_tools_.loadRemoteControl();

    // Clean up potential leftover collision objects from previous runs
    std::vector<std::string> old_object_ids = {"box1", "cylinder1"};
    planning_scene_interface_.removeCollisionObjects(old_object_ids);

    text_pose.translation().z() = 1.0;

    RCLCPP_INFO(node_->get_logger(), "MoveGroup Interface ready.");
}

void MoveGroupApp::showLogger(){
    RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", move_group_.getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", move_group_.getEndEffectorLink().c_str());
    RCLCPP_INFO(node_->get_logger(), "Available Planning Groups:");
    std::copy(move_group_.getJointModelGroupNames().begin(), move_group_.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
}

bool MoveGroupApp::planToPose() {
    // Planning to Pose Goal
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    move_group_.setPoseTarget(target_pose1);
    bool success = (move_group_.plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(node_->get_logger(), "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    RCLCPP_INFO(node_->get_logger(), "Visualizing plan 1 as trajectory line");
    visual_tools_.publishAxisLabeled(target_pose1, "pose1");
    visual_tools_.publishText(text_pose, "Pose_Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_.publishTrajectoryLine(current_plan_.trajectory, joint_group_);
    visual_tools_.trigger();
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    /* Uncomment below line when working with a real robot */
    /* move_group_.move(); */

    // Enforce planning
    // Setting up the constraints
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "panda_link7";
    ocm.header.frame_id = "panda_link0";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    move_group_.setPathConstraints(constraints);

    // Move to the constraints abide start pose
    moveit::core::RobotState start_state(*move_group_.getCurrentState());
    geometry_msgs::msg::Pose start_pose2;
    start_pose2.orientation.w = 1.0;
    start_pose2.position.x = 0.55;
    start_pose2.position.y = -0.05;
    start_pose2.position.z = 0.8;
    start_state.setFromIK(joint_group_, start_pose2);
    move_group_.setStartState(start_state);

    move_group_.setPoseTarget(target_pose1);
    move_group_.setPlanningTime(15.0);

    success = (move_group_.plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node_->get_logger(), "Visualizing plan 3 (constraints) %s, ", success? "":"Failed");

    visual_tools_.deleteAllMarkers();
    visual_tools_.publishAxisLabeled(start_pose2, "start");
    visual_tools_.publishAxisLabeled(target_pose1, "goal");
    visual_tools_.publishText(text_pose, "Constrainted goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_.publishTrajectoryLine(current_plan_.trajectory, joint_group_);
    visual_tools_.trigger();
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    move_group_.clearPathConstraints();

    // move_group.setPoseTarget(target_pose1);
    /**
    * Once plan() succeeds, the current_plan_ (which is a moveit::planning_interface::MoveGroupInterface::Plan struct) contains:
    * start_state_	A snapshot of the robot's joints exactly when the plan started.
    * trajectory_	The "Meat" of the plan. A list of waypoints with positions, velocities, and accelerations.
    * planning_time_	How many seconds it took the CPU to find the path. 
    */
    return success;
}
bool MoveGroupApp::planToJointSpace() {
    moveit::core::RobotStatePtr current_state = move_group_.getCurrentState(10);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_group_, joint_group_positions);
    joint_group_positions[0] = -1.0;

    bool within_bounds = move_group_.setJointValueTarget(joint_group_positions);
    if(!within_bounds) {
        RCLCPP_INFO(node_->get_logger(), "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    }
    move_group_.setMaxAccelerationScalingFactor(0.05);
    move_group_.setMaxVelocityScalingFactor(0.05);

    bool success = (move_group_.plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node_->get_logger(),"Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    visual_tools_.deleteAllMarkers();
    visual_tools_.publishText(text_pose, "Joint Space Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_.publishTrajectoryLine(current_plan_.trajectory, joint_group_);
    visual_tools_.trigger();
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    return success;
}

bool MoveGroupApp::executePlan() {
    bool success = (move_group_.execute(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node_->get_logger(), "Execution %s", success ? "SUCCEEDED" : "FAILED");
    return success;
}

void MoveGroupApp::planCartesianPath() {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    moveit_msgs::msg::RobotTrajectory trajectory;

    geometry_msgs::msg::Pose start_pose2;
    start_pose2.orientation.w = 1.0;
    start_pose2.position.x = 0.55;
    start_pose2.position.y = -0.05;
    start_pose2.position.z = 0.8;

    geometry_msgs::msg::Pose target_pose3 = start_pose2;
    target_pose3.position.z -= 0.2;
    waypoints.push_back(target_pose3);
    target_pose3.position.y -= 0.2;
    waypoints.push_back(target_pose3);
    target_pose3.position.x += 0.2; target_pose3.position.y += 0.2; target_pose3.position.z -= 0.2;
    waypoints.push_back(target_pose3);

    const double eef_step = 0.01;
    double fraction = move_group_.computeCartesianPath(waypoints, eef_step, trajectory);
    RCLCPP_INFO(node_->get_logger(), "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    visual_tools_.deleteAllMarkers();
    visual_tools_.publishText(text_pose, "Cartesian Path", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
    for(std::size_t i = 0; i < waypoints.size(); ++i) {
        visual_tools_.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
    }
    visual_tools_.trigger();
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    /* move_group_.execute(trajectory); */
}


void MoveGroupApp::planWithObstacle() {

    move_group_.setStartState(*move_group_.getCurrentState());
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 0;
    pose.orientation.x = -1.0;
    pose.position.x = 0.7;
    pose.position.y = 0.0;
    pose.position.z = 0.59;
    move_group_.setPoseTarget(pose);

    bool success = (move_group_.plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node_->get_logger(), "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");
    
    visual_tools_.deleteAllMarkers();
    visual_tools_.publishText(text_pose, "Clear_Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_.publishAxisLabeled(pose, "goal");
    visual_tools_.publishTrajectoryLine(current_plan_.trajectory, joint_group_);
    visual_tools_.trigger();
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_.getPlanningFrame();
    collision_object.id = "box1";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 1.5;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.48; box_pose.position.y = 0.0; box_pose.position.z = 0.25;
    box_pose.orientation.w = 1.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface_.applyCollisionObjects(collision_objects);

    RCLCPP_INFO(node_->get_logger(), "Add an object into the world");

    visual_tools_.publishText(text_pose, "Add Obstacle", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_.trigger();
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    success = (move_group_.plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node_->get_logger(), "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
    visual_tools_.publishText(text_pose, "Obstacle_Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_.publishTrajectoryLine(current_plan_.trajectory, joint_group_);
    visual_tools_.trigger();
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

    // Attach cylinder to hold and plan
    moveit_msgs::msg::CollisionObject object_to_attach;
    object_to_attach.id = "cylinder1";

    shape_msgs::msg::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.2;
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

    object_to_attach.header.frame_id = move_group_.getEndEffectorLink();
    geometry_msgs::msg::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.z = 0.2;

    object_to_attach.primitives.push_back(cylinder_primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;
    planning_scene_interface_.applyCollisionObject(object_to_attach);

    RCLCPP_INFO(node_->get_logger(), "Attach the object to the robot");

    std::vector<std::string> touch_links;
    touch_links.push_back("panda_rightfinger");
    touch_links.push_back("panda_leftfinger");
    move_group_.attachObject(object_to_attach.id, "panda_hand", touch_links);

    visual_tools_.publishText(text_pose, "Object_attached_to_robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_.trigger();
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

    move_group_.setStartStateToCurrentState();
    success = (move_group_.plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node_->get_logger(), "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
    visual_tools_.publishTrajectoryLine(current_plan_.trajectory, joint_group_);
    visual_tools_.trigger();
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

    RCLCPP_INFO(node_->get_logger(), "Detach the object from the robot");
    move_group_.detachObject(object_to_attach.id);

    visual_tools_.deleteAllMarkers();
    visual_tools_.publishText(text_pose, "Object_detached_from_robot", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_.trigger();
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

    RCLCPP_INFO(node_->get_logger(), "Remove the objects from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    object_ids.push_back(object_to_attach.id);
    planning_scene_interface_.removeCollisionObjects(object_ids);

    visual_tools_.publishText(text_pose, "Objects_removed", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools_.trigger();
    visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("move_group_tutorial_node", node_options);

    // Multi-threading is required for MoveGroup to work properly
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    auto app = std::make_shared<MoveGroupApp>(node, "panda_arm");

    app->showLogger();

    if (app->planToPose()) {
        // app->executePlan();
    }

    if (app->planToJointSpace()) {
        // app->executePlan();
    }

    app->planCartesianPath();

    app->planWithObstacle();

    rclcpp::shutdown();
    return 0;
}