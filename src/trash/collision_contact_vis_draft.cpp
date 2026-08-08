#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/collision_detection/collision_tools.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometric_shapes/shapes.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>

class CollisionContactTutorial {
public:
    CollisionContactTutorial(const rclcpp::Node::SharedPtr& node);

    void runCollisionCheck();
    void addWorldObject();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;

    moveit::core::RobotModelPtr robot_model_;
    rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_pub_;
    planning_scene::PlanningScenePtr planning_scene_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    visualization_msgs::msg::MarkerArray collision_markers_;

    shapes::ShapePtr g_world_cube_shape;

    void publishMarkers(visualization_msgs::msg::MarkerArray& markers);
};

CollisionContactTutorial::CollisionContactTutorial(const rclcpp::Node::SharedPtr& node)
    : node_(node), logger_(node->get_logger()) {
    
    auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description");
    robot_model_ = robot_model_loader->getModel();

    robot_state_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 10);

    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("collision_contact_markers", 10);
}

void CollisionContactTutorial::addWorldObject() {
    // 1. Define the primitive message
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {0.1, 0.1, 0.1};

    // 2. Define the ROS Pose message
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.3;
    box_pose.position.z = 0.5;
    box_pose.orientation.w = 1.0;

    // 3. Convert ROS Pose to Eigen Isometry3d (Required by the World API)
    Eigen::Isometry3d eigen_pose;
    tf2::fromMsg(box_pose, eigen_pose);

    // 4. Update the global shape pointer and add to the world
    // Note: Using -> because planning_scene_ is a pointer
    g_world_cube_shape = std::make_shared<shapes::Box>(0.1, 0.1, 0.1);

    planning_scene_->getWorldNonConst()->addToObject(
        "world_cube", 
        g_world_cube_shape, 
        eigen_pose
    );
}

void CollisionContactTutorial::runCollisionCheck() {
    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;

    c_req.contacts = true;
    c_req.max_contacts = 100;
    c_req.max_contacts_per_pair = 5;

    moveit::core::RobotState& current_state = planning_scene_->getCurrentStateNonConst();
    current_state.setToRandomPositions();

    planning_scene_->checkCollision(c_req, c_res, current_state);

    if (c_res.collision) {
        RCLCPP_INFO(logger_, "COLLIDING! Contacts found: %d", (int)c_res.contact_count);
        
        std_msgs::msg::ColorRGBA color;
        color.r = 1.0; color.a = 0.8; // Semi-transparent red

        visualization_msgs::msg::MarkerArray markers;
        collision_detection::getCollisionMarkersFromContacts(markers, "panda_link0", c_res.contacts, color, rclcpp::Duration::from_seconds(0), 0.01);
        
        publishMarkers(markers);
    } else {
        RCLCPP_INFO(logger_, "Not colliding.");
        visualization_msgs::msg::MarkerArray empty_markers;
        publishMarkers(empty_markers);
    }
}

void CollisionContactTutorial::publishMarkers(visualization_msgs::msg::MarkerArray& markers) {
    for (auto& marker : collision_markers_.markers) {
        marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    if (!collision_markers_.markers.empty()) {
        marker_pub_->publish(collision_markers_);
    }

    collision_markers_ = markers;
    marker_pub_->publish(collision_markers_);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("collision_contact_tutorial_node");

    CollisionContactTutorial tutorial(node);
    tutorial.addWorldObject();

    rclcpp::WallRate loop_rate(1); 
    while (rclcpp::ok()) {
        tutorial.runCollisionCheck();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}