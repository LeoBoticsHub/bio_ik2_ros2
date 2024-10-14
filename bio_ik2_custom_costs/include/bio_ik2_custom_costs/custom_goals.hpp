/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2017, Philipp Sebastian Ruppel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef CUSTOM_GOALS_HPP
#define CUSTOM_GOALS_HPP

#include "bio_ik/goal.h"
#include "bio_ik/robot_info.h"

namespace bio_ik {

/**
 * @brief class defining a new Goal supported by BioIkKinematicsQueryOptions
 * 		executes a function that returns a cost value for a given pose, with respect to a given seed state
 */
class IKCostFnGoalSeed : public Goal {

	const geometry_msgs::msg::Pose pose_;
	const kinematics::KinematicsBase::IKCostFn function_;
	const moveit::core::RobotModelConstPtr robot_model_;
	const std::vector<double> seed_state_;

public:
	/**
	 * @brief constructor for the IKCostFnGoalSeed class
	 * @param pose - the target pose
	 * @param function - the cost function to be evaluated for a candidate IK solution
	 * @param robot_model - the robot model
	 * @param seed_state - a given seed state to evaluate the cost function with
	 * @param weight - the weight of the goal (default = 1.0)
	 */
	IKCostFnGoalSeed(const geometry_msgs::msg::Pose &pose,
					 const kinematics::KinematicsBase::IKCostFn &function,
					 const moveit::core::RobotModelConstPtr &robot_model,
					 const std::vector<double> &seed_state,
					 double weight = 1.0);

	/**
	 * @brief function to evaluate the cost of the goal
	 * @param context - the goal context: extract information about robot state, joint model group, and robot model
	 * @return the cost of the goal
	 */
	double evaluate(const GoalContext &context) const override;
};

/**
 * @brief class defining a new Goal supported by BioIkKinematicsQueryOptions
 * 		minimizes the displacement of the robot's joints from a given seed state
 */
class MinimalDisplacementGoalSeed : public Goal {
private:
	// fixed seed state
	const std::vector<double> seed_state_;

public:
	/**
	 * @brief constructor for the MinimalDisplacementGoalSeed class
	 * 		gives high cost to solutions that are far from the seed state
	 * @param seed_state - the seed state to minimize the displacement from
	 * @param weight - the weight of the goal (default = 1.0)
	 * @param secondary - the secondary goal flag (default = true)
	 */
	MinimalDisplacementGoalSeed(const std::vector<double> &seed_state, double weight = 1.0, bool secondary = true);

	/**
	 * @brief function to evaluate the cost of the goal
	 * @param context - the goal context: extract information about robot state, joint model group, and robot model
	 * @return the cost of the goal
	 */
	double evaluate(const GoalContext &context) const;
};

//
/**
 * @brief class defining a new Goal supported by BioIkKinematicsQueryOptions
 * ConfigureElbowGoal tries to keep the elbow joint in the center half of the specified joint limits
 * If the elbow joint is in the center of the specified joint limits ((upper_limit_ + lower_limit_) * 0.5) , the cost is 0
 * If the elbow joint is at the upper or lower limit, the cost is proportional to the distance from the center
 * The result is double and reduced by the half-span of the joint's range ((upper_limit_ - lower_limit_) * 0.5).
 * This operation centers the deviation around zero.
 */
class ConfigureElbowGoal : public Goal {
private:
	const double lower_limit_;
	const double upper_limit_;
	const int joint_elbow_index_;

public:
	/**
	 * @brief Constructor for the ConfigureElbowGoal class
	 * @param joint_elbow_index - the index of the elbow joint
	 * @param lower_limit - the lower limit of the elbow joint
	 * @param upper_limit - the upper limit of the elbow joint
	 * @param weight - the weight of the goal (default = 1.0)
	 */
	ConfigureElbowGoal(const int joint_elbow_index, const double lower_limit, const double upper_limit, double weight = 1.0);

	/**
	 * @brief Evaluate the cost of the goal
	 * @param context - the goal context: extract information about robot state, joint model group, and robot model
	 * @return the cost of the goal
	 */
	double evaluate(const GoalContext &context) const;
};

/**
 * @brief class defining a new Goal supported by BioIkKinematicsQueryOptions
 * MaxManipulabilityGoal tries to maximize the manipulability
 * if svd == true, the singular value decomposition is used to compute the manipulability:
 * 		Compute the the condition number (The inverse of the condition number is a measure of the manipulability)
 * 		As this cost will be minimized and we want to maximize manipulability, we return the condition number*weight
 * 		A high inverse of the condition number means a high manipulability -> a low condition number implies a high manipulability
 * if svd == false, the manipulability is computed as the square root of the determinant of the Jacobian times its transpose
 * 		its inverse is minimized
 */
class MaxManipulabilityGoal : public Goal {
private:
	const Eigen::MatrixXd jacobian_;
	bool svd_;

public:
	/**
	 * @brief Constructor for the MaxManipulabilityGoal class
	 * @param jacobian - the Jacobian matrix obtained from the robot state
	 * @param svd - flag to use the singular value decomposition to compute the manipulability
	 * @param weight - the weight of the goal (default = 1.0)
	 */
	MaxManipulabilityGoal(const Eigen::MatrixXd jacobian, bool svd, double weight = 1.0);

	/**
	 * @brief Evaluate the cost of the goal
	 * @param context - the goal context: extract information about robot state, joint model group, and robot model
	 * @return the cost of the goal
	 */
	double evaluate(const GoalContext &context) const;
};

/**
 * @brief class defining a new Goal supported by BioIkKinematicsQueryOptions
 * MinimalVelocityJointGoal tries to keep the velocity of a joint under its joint velocity limit
 */
class MinimalVelocityJointGoal : public Goal {
private:
	double time_step_;
	int joint_index_;

public:
	/**
	 * @brief Constructor for the MinimalVelocityJointGoal class
	 * @param time_step - the time step to compute the velocity
	 * @param joint_index - the index of the joint to keep the velocity under the maximum value
	 * @param weight - the weight of the goal (default = 1.0)
	 */
	MinimalVelocityJointGoal(double time_step, int joint_index, double weight = 1.0);

	/**
	 * @brief Evaluate the cost of the goal
	 * @param context - the goal context: extract information about robot state, joint model group, and robot model
	 * @return the cost of the goal
	 */
	double evaluate(const GoalContext &context) const;
};

/**
 * @brief class defining a new Goal supported by BioIkKinematicsQueryOptions
 * MinimalAccelerationGoal tries to keep the acceleration of all joint under the joint acceleration limits
 */
class MinimalAccelerationGoal : public Goal {
private:
	const std::vector<double> acceleration_limits_;
	double time_step_;

public:
	/**
	 * @brief Constructor for the MinimalAccelerationGoal class
	 * @param acceleration_limits - the acceleration limits for all joints
	 * @param time_step - the time step to compute the acceleration
	 * @param weight - the weight of the goal (default = 1.0)
	 */
	MinimalAccelerationGoal(const std::vector<double> acceleration_limits, double time_step, double weight = 1.0);

	/**
	 * @brief Evaluate the cost of the goal
	 * @param context - the goal context: extract information about robot state, joint model group, and robot model
	 * @return the cost of the goal
	 */
	double evaluate(const GoalContext &context) const;
};

/**
 * @brief This goal combines multiple goals at once
 * Unique goal that computes the total sum of the costs of different cost functions with their respective weights
 * The cost of the goals is computed by the evaluate function
 * The goal can be configured to apply the following goals:
 * 	- minimal displacement goal with respect to the given seed (current robot state)
 * 	- linear cost to prefer solutions in the joints center and avoid joint limits
 * 	- cost to enforce virtual hard limits on one joint, to prevent strange solutions
 * 	- cost to enable the manipulability goal
 * 	- cost to keep velocity of a joint under maximum value
 * 	- cost to keep joints acceleration under maximum value
 */
class MultipleGoalsAtOnce : public Goal {
private:
	// minimal displacement goal with respect to the given seed (= current robot state)
	bool apply_minimal_displacement_goal_;
	// linear cost to prefer solutions in the joints center and avoid joint limits
	bool apply_avoid_joint_limits_goal_;
	// cost to enforce virtual hard limits on one joint, to prevent strange solutions
	bool apply_hard_limits_goal_;
	// cost to enable the manipulability goal
	bool apply_manipulability_goal_;
	// cost to keep velocity of a joint under maximum value
	bool apply_min_velocity_goal_;
	// cost to keep joints acceleration under maximum value
	bool apply_min_acceleration_goal_;

	// Jacobian matrix
	Eigen::MatrixXd jacobian_;

	// weights for the goals
	double w_manipulability_;
	double w_minimum_displacement_;
	double w_avoid_joint_limits_;
	double w_hard_limits_;
	std::vector<double> w_min_velocities_;
	double w_min_acceleration_;

	// hard limits goal parameters
	// elbow
	double lower_limit_;
	double upper_limit_;
	int limited_joint_index_;

	// minimal velocity joint goal parameters
	double time_step_;
	std::vector<int> joint_indeces_;

	// minimal acceleration goal parameters
	std::vector<double> acceleration_limits_;

public:
	/**
	 * @brief Constructor for the MultipleGoalsAtOnce class
	 */
	MultipleGoalsAtOnce();

	/**
	 * @brief Apply the minimal displacement goal, and sets the relative flag to true
	 * @param weight - the weight of the goal (default = 1.0)
	 */
	void applyMinimalDisplacementGoal(double weight = 1.0);

	/**
	 * @brief Apply the avoid joint limits goal, and sets the relative flag to true
	 * @param weight - the weight of the goal (default = 1.0)
	 */
	void applyAvoidJointLimitsGoal(double weight = 1.0);

	/**
	 * @brief Apply the hard limits goal, and sets the relative flag to true
	 * @param lower_limit - the lower limit of the joint
	 * @param upper_limit - the upper limit of the joint
	 * @param joint_index - the index of the joint
	 * @param weight - the weight of the goal (default = 1.0)
	 */
	void applyHardLimitsGoal(double lower_limit, double upper_limit, int joint_index, double weight = 1.0);

	/**
	 * @brief Apply the manipulability goal, and sets the relative flag to true
	 * @param jacobian - the Jacobian matrix obtained from the robot state
	 * @param weight - the weight of the goal (default = 1.0)
	 */
	void applyManipulabilityGoal(const Eigen::MatrixXd jacobian, double weight = 1.0);

	/**
	 * @brief Apply the minimal velocity joint goal, and sets the relative flag to true
	 * @param time_step - the time step to compute the velocity
	 * @param joint_indeces - the indices of the joints to keep the velocity under the maximum value
	 * @param weights - the weights of the goal
	 */
	void applyMinimalVelocitiesGoal(double time_step, std::vector<int> joint_indeces, std::vector<double> weights);

	/**
	 * @brief Apply the minimal acceleration goal, and sets the relative flag to true
	 * @param acceleration_limits - the acceleration limits for all joints
	 * @param time_step - the time step to compute the acceleration
	 * @param weight - the weight of the goal (default = 1.0)
	 */
	void applyMinimalAccelerationCost(const std::vector<double> acceleration_limits, double time_step, double weight = 1.0);

	/**
	 * @brief Evaluate the cost of the goal
	 * @param context - the goal context: extract information about robot state, joint model group, and robot model
	 * @return the cost of the goal
	 */
	double evaluate(const bio_ik::GoalContext &context) const override;
};

} // namespace bio_ik

#endif