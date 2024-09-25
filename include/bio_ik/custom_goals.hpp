#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <Eigen/Dense>


class CustomGoal {
public:

    // weight of the cost functions
    double weight_l2_norm = 0;
    double weight_manip = 0;
   
    // Constructor to initialize the weights
    CustomGoal(double weight_l2_norm = 0.0, double weight_manip = 0.0);
    
    /////////////////////////////////////////////////////////////
    // Cost Functions for the custom goals used in the IK call //
    /////////////////////////////////////////////////////////////

    // The cost function will optimize for minimal joint movement.
    double cost_fn_l2_norm(const geometry_msgs::msg::Pose&, /*goal_pose*/
                           const moveit::core::RobotState& solution_state,
                           const moveit::core::JointModelGroup* jmg,
                           const std::vector<double>& initial_guess) const;

    // The cost function will optimize for maximum manipulability.
    double cost_fn_manip(const geometry_msgs::msg::Pose&, /*goal_pose*/
                         const moveit::core::RobotState& solution_state,
                         const moveit::core::JointModelGroup* jmg,
                         const std::vector<double>& initial_guess) const;

    ///////////////////////////////////////////////////////////////
    // Helper functions used in the computation of custom goals. //
    ///////////////////////////////////////////////////////////////

    // Compute the L2 norm of the difference between the solution and the start joint positions
    double compute_l2_norm(const std::vector<double>& solution, const std::vector<double>& start) const;

    // Compute the manipulability of the solution
    double compute_manip(Eigen::MatrixXd jacobian) const;
};

