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

#include "custom_goals.hpp"

using namespace bio_ik;

/**
 * @brief constructor for the IKCostFnGoalSeed class
 * @param pose - the target pose
 * @param function - the cost function to be evaluated for a candidate IK solution
 * @param robot_model - the robot model
 * @param seed_state - a given seed state to evaluate the cost function with
 * @param weight - the weight of the goal (default = 1.0)
 */
IKCostFnGoalSeed::IKCostFnGoalSeed(const geometry_msgs::msg::Pose &pose,
								   const kinematics::KinematicsBase::IKCostFn &function,
								   const moveit::core::RobotModelConstPtr &robot_model,
								   const std::vector<double> &seed_state,
								   double weight)
	: Goal(),
	  pose_(pose),
	  function_(function),
	  robot_model_(robot_model),
	  seed_state_(seed_state) {
	// set the weight of the goal
	setWeight(weight);
}

/**
 * @brief function to evaluate the cost of the goal
 * @param context - the goal context: extract information about robot state, joint model group, and robot model
 * @return the cost of the goal
 */
double IKCostFnGoalSeed::evaluate(const GoalContext &context) const {

	auto info = context.getRobotInfo();
	moveit::core::RobotState robot_state(robot_model_);
	auto jmg = context.getJointModelGroup();

	// copy the temporary solution to the position vector
	std::vector<double> sol_positions(context.getProblemVariableCount());
	for (size_t i = 0; i < context.getProblemVariableCount(); ++i) {
		sol_positions[i] = context.getProblemVariablePosition(i);
	}

	robot_state.setJointGroupPositions(&jmg, sol_positions);
	robot_state.update();
	return function_(pose_, robot_state, &jmg, seed_state_);
}

/**
 * @brief constructor for the MinimalDisplacementGoalSeed class
 * 		gives high cost to solutions that are far from the seed state
 * @param seed_state - the seed state to minimize the displacement from
 * @param weight - the weight of the goal (default = 1.0)
 * @param secondary - the secondary goal flag (default = true)
 */
MinimalDisplacementGoalSeed::MinimalDisplacementGoalSeed(const std::vector<double> &seed_state,
														 double weight, bool secondary)
	: seed_state_(seed_state) {
	weight_ = weight;
	secondary_ = secondary;
}

/**
 * @brief function to evaluate the cost of the goal
 * @param context - the goal context: extract information about robot state, joint model group, and robot model
 * @return the cost of the goal
 */
double MinimalDisplacementGoalSeed::evaluate(const GoalContext &context) const {
	double sum = 0.0;
	for (size_t i = 0; i < context.getProblemVariableCount(); i++) {
		double d = context.getProblemVariablePosition(i) - seed_state_[i];
		sum += d * d;
	}
	return sum * weight_;
}

/**
 * @brief Constructor for the HardJointLimitsGoal class
 * @param joint_index - the index of the joint
 * @param lower_limit - the lower limit of the joint
 * @param upper_limit - the upper limit of the joint
 * @param weight - the weight of the goal (default = 1.0)
 */
HardJointLimitsGoal::HardJointLimitsGoal(const int joint_index, const double lower_limit,
										 const double upper_limit, double weight)
	: lower_limit_(lower_limit),
	  upper_limit_(upper_limit),
	  joint_index_(joint_index) {
	secondary_ = true;
	weight_ = weight;
}

/**
 * @brief Evaluate the cost of the goal
 * @param context - the goal context: extract information about robot state, joint model group, and robot model
 * @return the cost of the goal
 */
double HardJointLimitsGoal::evaluate(const GoalContext &context) const {
	double sum = 0.0;

	double d = context.getProblemVariablePosition(joint_index_) - (upper_limit_ + lower_limit_) * 0.5;
	if (d > (upper_limit_ - lower_limit_) * 0.5) {
		d = 100.0;
	} else {
		d = fmax(0.0, fabs(d) * 2.0 - (upper_limit_ - lower_limit_) * 0.5);
	}
	sum += d * d;

	return sum * weight_;
}

/**
 * @brief Constructor for the MaxManipulabilityGoal class
 * @param jacobian - the Jacobian matrix obtained from the robot state
 * @param svd - flag to use the singular value decomposition to compute the manipulability
 * @param weight - the weight of the goal (default = 1.0)
 */
MaxManipulabilityGoal::MaxManipulabilityGoal(const Eigen::MatrixXd jacobian, bool svd, double weight)
	: jacobian_(jacobian),
	  svd_(svd) {
	weight_ = weight;
	secondary_ = true;
}

/**
 * @brief Evaluate the cost of the goal
 * @param context - the goal context: extract information about robot state, joint model group, and robot model
 * @return the cost of the goal
 */
double MaxManipulabilityGoal::evaluate(const GoalContext & /*context*/) const {
	Eigen::VectorXd singular_values;
	double condition_number = 0;
	double sum = 0.0;
	double min_sv = 0.0;

	if (svd_) {
		// compute the singular values of the Jacobian
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian_, Eigen::ComputeThinU | Eigen::ComputeThinV);
		singular_values = svd.singularValues();

		// Compute the the condition number (The inverse of the condition number is a measure of the manipulability)
		if (singular_values.minCoeff() == 0) {
			min_sv = 1e-6;
		} else {
			min_sv = singular_values.minCoeff();
		}

		condition_number = singular_values.maxCoeff() / min_sv;
		sum += condition_number * condition_number;

		return sum * weight_;
	} else {
		// Compute the manipulability
		double manipulability = sqrt((jacobian_ * jacobian_.transpose()).determinant());
		if (manipulability == 0) {
			manipulability = 1e-6;
		}

		return weight_ / manipulability;
	}
}

/**
 * @brief Constructor for the DesiredVelocityJointGoal class
 * @param time_step - the time step to compute the velocity
 * @param scale - the scale to multiply the maximum velocity, in [0,1]
 * @param joint_indeces - the indeces of the joint to keep the velocity under the maximum value
 * @param previous_joint_positions - the previous joint positions used to compute the velocity
 * @param weights - the weights for the cost for each joint index
 */
DesiredVelocityJointGoal::DesiredVelocityJointGoal(double time_step, double scale, std::vector<int> joint_indeces,
												   std::vector<double> previous_joint_positions, std::vector<double> weights)
	: time_step_(time_step),
	  scale_(scale),
	  joint_indeces_(joint_indeces),
	  previous_joint_positions_(previous_joint_positions),
	  weights_(weights) {
	secondary_ = true;
	// treshold the value of the scale parameter in [0,1]
	if (scale_ < 0) {
		scale_ = 0;
	}
	if (scale_ > 1) {
		scale_ = 1;
	}
}

/**
 * @brief Evaluate the cost of the goal
 * @param context - the goal context: extract information about robot state, joint model group, and robot model
 * @return the cost of the goal
 */
double DesiredVelocityJointGoal::evaluate(const GoalContext &context) const {
	double sum = 0.0;
	auto &info = context.getRobotInfo();

	for (unsigned int i = 0; i < joint_indeces_.size(); i++) {
		double velocity_limit_ = info.getMaxVelocity(joint_indeces_[i]);
		double d = context.getProblemVariablePosition(joint_indeces_[i]) - previous_joint_positions_[i];
		double vel_d = fmax(0.0, fabs(d) / time_step_ - velocity_limit_ * scale_);
		sum += vel_d * vel_d * weights_[i];
	}

	return sum * weight_;
}

/**
 * @brief Constructor for the MultipleGoalsAtOnce class
 */
MultipleGoalsAtOnce::MultipleGoalsAtOnce() {
	secondary_ = true;
	weight_ = 1.0;
	apply_avoid_joint_limits_goal_ = false;
	apply_minimal_displacement_goal_ = false;
	apply_hard_limits_goal_ = false;
	apply_manipulability_goal_ = false;
	apply_des_velocity_goal_ = false;
}

/**
 * @brief Apply the minimal displacement goal, and sets the relative flag to true
 * @param weight - the weight of the goal (default = 1.0)
 */
void MultipleGoalsAtOnce::applyAvoidJointLimitsGoal(double weight) {
	w_avoid_joint_limits_ = weight;
	apply_avoid_joint_limits_goal_ = true;
}

/**
 * @brief Apply the avoid joint limits goal, and sets the relative flag to true
 * @param weight - the weight of the goal (default = 1.0)
 */
void MultipleGoalsAtOnce::applyMinimalDisplacementGoal(double weight) {
	w_minimum_displacement_ = weight;
	apply_minimal_displacement_goal_ = true;
}

/**
 * @brief Apply the hard limits goal, and sets the relative flag to true
 * @param lower_limit - the lower limit of the joint
 * @param upper_limit - the upper limit of the joint
 * @param joint_index - the index of the joint
 * @param weight - the weight of the goal (default = 1.0)
 */
void MultipleGoalsAtOnce::applyHardLimitsGoal(double lower_limit, double upper_limit, int joint_index, double weight) {
	w_hard_limits_ = weight;
	apply_hard_limits_goal_ = true;
	lower_limit_ = lower_limit;
	upper_limit_ = upper_limit;
	limited_joint_index_ = joint_index;
}

/**
 * @brief Apply the manipulability goal, and sets the relative flag to true
 * @param jacobian - the Jacobian matrix obtained from the robot state
 * @param weight - the weight of the goal (default = 1.0)
 */
void MultipleGoalsAtOnce::applyManipulabilityGoal(const Eigen::MatrixXd jacobian, double weight) {
	jacobian_ = jacobian;
	w_manipulability_ = weight;
	apply_manipulability_goal_ = true;
}

/**
 * @brief Apply the desired velocity joint goal, and sets the relative flag to true
 * @param time_step - the time step to compute the velocity
 * @param scale - the scale to multiply the maximum velocity, in [0,1]
 * @param joint_indeces - the indices of the joints to keep the velocity under the maximum value
 * @param previous_joint_positions - the previous joint positions used to compute the velocity
 * @param weights - the weights of the goal
 */
void MultipleGoalsAtOnce::applyDesiredVelocitiesGoal(double time_step, double scale, std::vector<int> joint_indeces,
													 std::vector<double> previous_joint_positions,
													 std::vector<double> weights) {
	time_step_ = time_step;
	scale_ = scale;
	joint_indeces_ = joint_indeces;
	previous_joint_positions_ = previous_joint_positions;
	w_des_velocities_ = weights;
	apply_des_velocity_goal_ = true;

	// treshold the value of the scale parameter in [0,1]
	if (scale_ < 0) {
		scale_ = 0;
	}
	if (scale_ > 1) {
		scale_ = 1;
	}
}

/**
 * @brief Evaluate the cost of the goal
 * @param context - the goal context: extract information about robot state, joint model group, and robot model
 * @return the cost of the goal
 */
double MultipleGoalsAtOnce::evaluate(const bio_ik::GoalContext &context) const {
	double sum = 0.0;

	// minimal displacement goal
	if (apply_minimal_displacement_goal_) {
		for (size_t i = 0; i < context.getProblemVariableCount(); i++) {
			double d = context.getProblemVariablePosition(i) - context.getProblemVariableInitialGuess(i);
			sum += d * d * w_minimum_displacement_;
		}
	}

	// avoid joint limits goal
	if (apply_avoid_joint_limits_goal_) {
		auto &info = context.getRobotInfo();
		for (size_t i = 0; i < context.getProblemVariableCount(); i++) {
			size_t ivar = context.getProblemVariableIndex(i);
			if (info.getClipMax(ivar) == DBL_MAX)
				continue;
			double d = context.getProblemVariablePosition(i) - (info.getMin(ivar) + info.getMax(ivar)) * 0.5;
			d = fmax(0.0, fabs(d) * 2.0 - info.getSpan(ivar) * 0.5);
			sum += d * d * w_avoid_joint_limits_;
		}
	}

	// hard limits goal
	if (apply_hard_limits_goal_) {
		double d = context.getProblemVariablePosition(limited_joint_index_) - (upper_limit_ + lower_limit_) * 0.5;
		d = fmax(0.0, fabs(d) * 2.0 - (upper_limit_ - lower_limit_) * 0.5);
		sum += d * d * w_hard_limits_;
	}

	// manipulability goal
	if (apply_manipulability_goal_) {
		Eigen::VectorXd singular_values;
		double condition_number = 0.0;
		// double sum = 0.0;
		double min_sv = 0.0;

		if (svd_) {
			// compute the singular values of the Jacobian
			Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian_, Eigen::ComputeThinU | Eigen::ComputeThinV);
			singular_values = svd.singularValues();

			// Compute the the condition number (The inverse of the condition number is a measure of the manipulability)
			if (singular_values.minCoeff() == 0) {
				min_sv = 1e-6;
			} else {
				min_sv = singular_values.minCoeff();
			}

			condition_number = singular_values.maxCoeff() / min_sv;
			sum += condition_number * condition_number * w_manipulability_;
		} else {
			// Compute the manipulability with the alternative method
			double manipulability = sqrt((jacobian_ * jacobian_.transpose()).determinant());
			if (manipulability == 0) {
				manipulability = 1e-6;
			}

			sum += w_manipulability_ / manipulability;
		}
	}

	// desired velocity joint goal
	if (apply_des_velocity_goal_) {
		auto &info = context.getRobotInfo();
		for (unsigned int i = 0; i < joint_indeces_.size(); i++) {
			double velocity_limit_ = info.getMaxVelocity(joint_indeces_[i]);
			double d = context.getProblemVariablePosition(joint_indeces_[i]) - previous_joint_positions_[i];
			double vel_d = fmax(0.0, fabs(d) / time_step_ - velocity_limit_ * scale_);
			sum += vel_d * vel_d * w_des_velocities_[i];
		}
	}

	return sum;
}
