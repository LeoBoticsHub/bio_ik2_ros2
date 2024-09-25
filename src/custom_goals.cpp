#include <vector>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <bio_ik/custom_goals.hpp>

// Constructor definition
CustomGoal::CustomGoal(double weight_l2_norm, double weight_manip)
    : weight_l2_norm(weight_l2_norm), weight_manip(weight_manip) {}

/////////////////////////////////////////////////////////////
// Cost Functions for the custom goals used in the IK call //
/////////////////////////////////////////////////////////////

// The cost function will optimize for minimal joint movement.
double CustomGoal::cost_fn_l2_norm(const geometry_msgs::msg::Pose& /*goal_pose*/,
                        const moveit::core::RobotState& solution_state,
                        const moveit::core::JointModelGroup* jmg,
                        const std::vector<double>& initial_guess) const {
    std::vector<double> proposed_joint_positions;
    solution_state.copyJointGroupPositions(jmg, proposed_joint_positions);
    double cost = compute_l2_norm(proposed_joint_positions, initial_guess);
    return weight_l2_norm * cost;
}

// The cost function will optimize for maximum manipulability.
double CustomGoal::cost_fn_manip(const geometry_msgs::msg::Pose&, /*goal_pose*/
                         const moveit::core::RobotState& solution_state,
                         const moveit::core::JointModelGroup* jmg,
                         const std::vector<double>& initial_guess) const {

    auto jacobian = solution_state.getJacobian(jmg);
    // associate low manipulability with high cost
    return weight_manip / compute_manip(jacobian);                        
}

///////////////////////////////////////////////////////////////
// Helper functions used in the computation of custom goals. //
///////////////////////////////////////////////////////////////

// Compute the L2 norm of the difference between the solution and the start joint positions
double CustomGoal::compute_l2_norm(const std::vector<double>& solution, const std::vector<double>& start) const {
    double sum = 0.0;
    for (size_t ji = 0; ji < solution.size(); ji++) {
        double d = solution[ji] - start[ji];
        sum += d * d;
    }
    return sum;
}

// Compute the manipulability of the solution
double CustomGoal::compute_manip(Eigen::MatrixXd jacobian) const {
    return sqrt((jacobian * jacobian.transpose()).determinant());
};