#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

/**
* @brief Constructor that initializes the RobotModel and RobotState
* @param node The ROS 2 node used for parameter loading
*/
class RobotModelStateTutorial {
public:
    RobotModelStateTutorial(const rclcpp::Node::SharedPtr& node);

    void printBasicInfo();
    void printJointValues(const std::string& group_name);
    void checkJointLimits(const std::string& group_name);
    void computeFK(const std::string& group_name, const std::string& link_name);
    void solveIK(const std::string& group_name, const std::string& link_name);
    void getJacobian(const std::string& group_name);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr robot_state_;
};

RobotModelStateTutorial::RobotModelStateTutorial(const rclcpp::Node::SharedPtr& node)
    : node_(node), logger_(node->get_logger()) {
    
    robot_model_loader::RobotModelLoader robot_model_loader(node_);
    kinematic_model_ = robot_model_loader.getModel();
    
    if (!kinematic_model_) {
        RCLCPP_ERROR(logger_, "Could not load robot model");
        return;
    }

    robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    robot_state_->setToDefaultValues();
}

void RobotModelStateTutorial::printBasicInfo() {
    RCLCPP_INFO(logger_, "Model frame: %s", kinematic_model_->getModelFrame().c_str());
}

void RobotModelStateTutorial::printJointValues(const std::string& group_name) {
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup(group_name);
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    std::vector<double> joint_values;
    robot_state_->copyJointGroupPositions(joint_model_group, joint_values);

    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        RCLCPP_INFO(logger_, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
}

void RobotModelStateTutorial::checkJointLimits(const std::string& group_name) {
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup(group_name);
    
    // Force an invalid value
    std::vector<double> joint_values;
    robot_state_->copyJointGroupPositions(joint_model_group, joint_values);
    joint_values[0] = 5.57; 
    robot_state_->setJointGroupPositions(joint_model_group, joint_values);

    RCLCPP_INFO_STREAM(logger_, "Current state is " << (robot_state_->satisfiesBounds() ? "valid" : "not valid"));
    
    robot_state_->enforceBounds();
    RCLCPP_INFO_STREAM(logger_, "After enforcing bounds: " << (robot_state_->satisfiesBounds() ? "valid" : "not valid"));
}

void RobotModelStateTutorial::computeFK(const std::string& group_name, const std::string& link_name) {
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup(group_name);
    robot_state_->setToRandomPositions(joint_model_group);
    
    const Eigen::Isometry3d& end_effector_state = robot_state_->getGlobalLinkTransform(link_name);
    RCLCPP_INFO_STREAM(logger_, "Translation: \n" << end_effector_state.translation() << "\n");
}

void RobotModelStateTutorial::solveIK(const std::string& group_name, const std::string& link_name) {
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup(group_name);
    const Eigen::Isometry3d& end_effector_state = robot_state_->getGlobalLinkTransform(link_name);

    double timeout = 0.1;
    bool found_ik = robot_state_->setFromIK(joint_model_group, end_effector_state, timeout);

    if (found_ik) {
        RCLCPP_INFO(logger_, "IK solution found successfully.");
    } else {
        RCLCPP_INFO(logger_, "Did not find IK solution");
    }
}

void RobotModelStateTutorial::getJacobian(const std::string& group_name) {
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup(group_name);
    
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    robot_state_->getJacobian(joint_model_group, 
                             robot_state_->getLinkModel(joint_model_group->getLinkModelNames().back()),
                             reference_point_position, jacobian);
                             
    RCLCPP_INFO_STREAM(logger_, "Jacobian: \n" << jacobian << "\n");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("robot_model_state_tutorial_node", node_options);

    RobotModelStateTutorial tutorial(node);
    
    tutorial.printBasicInfo();
    tutorial.printJointValues("panda_arm");
    tutorial.checkJointLimits("panda_arm");
    tutorial.computeFK("panda_arm", "panda_link8");
    tutorial.solveIK("panda_arm", "panda_link8");
    tutorial.getJacobian("panda_arm");

    rclcpp::shutdown();
    return 0;
}