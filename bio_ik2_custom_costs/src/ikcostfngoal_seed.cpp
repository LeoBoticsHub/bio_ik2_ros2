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

#include "ikcostfngoal_seed.hpp"

using namespace bio_ik;

IKCostFnGoalSeed::IKCostFnGoalSeed(const geometry_msgs::msg::Pose &pose,
								   const kinematics::KinematicsBase::IKCostFn &function,
								   const moveit::core::RobotModelConstPtr &robot_model,
								   const std::vector<double> &seed_state,
								   double weight)
	: Goal(),
	  pose_(pose),
	  function_(function),
	  robot_model_(robot_model),
	  seed_state_(seed_state)
{
	// set the weight of the goal
	setWeight(weight);
}

double IKCostFnGoalSeed::evaluate(const GoalContext &context) const
{

	auto info = context.getRobotInfo();
	moveit::core::RobotState robot_state(robot_model_);
	auto jmg = context.getJointModelGroup();

	// copy the temporary solution to the position vector
	std::vector<double> sol_positions(context.getProblemVariableCount());
	for (size_t i = 0; i < context.getProblemVariableCount(); ++i)
	{
		sol_positions[i] = context.getProblemVariablePosition(i);
	}

	robot_state.setJointGroupPositions(&jmg, sol_positions);
	robot_state.update();
	return function_(pose_, robot_state, &jmg, seed_state_);
}

MinimalDisplacementGoalSeed::MinimalDisplacementGoalSeed(const std::vector<double> &seed_state,
														 double weight, bool secondary)
	: seed_state_(seed_state)
{
	weight_ = weight;
	secondary_ = secondary;
}

double MinimalDisplacementGoalSeed::evaluate(const GoalContext &context) const
{
	double sum = 0.0;
	for (size_t i = 0; i < context.getProblemVariableCount(); i++)
	{
		double d = context.getProblemVariablePosition(i) - seed_state_[i];
		d *= weight_;
		sum += d * d;
	}
	return sum;
}

ConfigureElbowGoal::ConfigureElbowGoal(const int joint_elbow_index, double lower_limit,
									   double upper_limit, double weight)
	: lower_limit_(lower_limit),
	  upper_limit_(upper_limit),
	  joint_elbow_index_(joint_elbow_index)
{
	secondary_ = true;
	weight_ = weight;
}

double ConfigureElbowGoal::evaluate(const GoalContext &context) const
{
	double sum = 0.0;

	double d = context.getProblemVariablePosition(joint_elbow_index_) - (upper_limit_ + lower_limit_) * 0.5;
	d = fmax(0.0, fabs(d) * 2.0 - (upper_limit_ - lower_limit_) * 0.5);
	d *= weight_;
	sum += d * d;

	return sum;
}

MaxManipulabilityGoal::MaxManipulabilityGoal(const Eigen::MatrixXd jacobian, bool svd, double weight)
	: jacobian_(jacobian),
	  svd_(svd)
{
	weight_ = weight;
	secondary_ = true;
}

double MaxManipulabilityGoal::evaluate(const GoalContext &context) const
{
	Eigen::VectorXd singular_values;
	double condition_number = 0;
	double sum = 0.0;
	double min_sv = 0.0;

	if (svd_)
	{
		// compute the singular values of the Jacobian
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian_, Eigen::ComputeThinU | Eigen::ComputeThinV);
		singular_values = svd.singularValues();

		// Compute the the condition number (The inverse of the condition number is a measure of the manipulability)
		if (singular_values.minCoeff() == 0)
		{
			min_sv = 1e-6;
		}
		else
		{
			min_sv = singular_values.minCoeff();
		}

		condition_number = singular_values.maxCoeff() / min_sv;
		condition_number *= weight_;
		sum += condition_number * condition_number;

		return sum;
	}
	else
	{
		// Compute the manipulability
		double manipulability = sqrt((jacobian_ * jacobian_.transpose()).determinant());
		if (manipulability == 0)
		{
			manipulability = 1e-6;
		}

		return (weight_ * weight_) / manipulability;
	}
}

MinimalVelocityjointGoal::MinimalVelocityjointGoal(double time_step, int joint_index, double weight)
    : time_step_(time_step),
	  joint_index_(joint_index)
{
	weight_ = weight;
	secondary_ = true;
}

double MinimalVelocityjointGoal::evaluate(const GoalContext &context) const
{
	double sum = 0.0;
	auto &info = context.getRobotInfo();
	double velocity_limit_ = info.getMaxVelocity(joint_index_);
	double d = context.getProblemVariablePosition(joint_index_) - context.getProblemVariableInitialGuess(joint_index_);
	double vel_d = fmax(0.0, fabs(d) / time_step_ - velocity_limit_);
	vel_d *= weight_;
	sum += vel_d * vel_d;

	return sum;
}

MinimalAccelerationGoal::MinimalAccelerationGoal(const std::vector<double> acceleration_limits, double time_step, double weight)
	: acceleration_limits_(acceleration_limits),
	  time_step_(time_step)
{
	weight_ = weight;
	secondary_ = true;
}

double MinimalAccelerationGoal::evaluate(const GoalContext &context) const
{
	double sum = 0.0;
	for (size_t i = 0; i < context.getProblemVariableCount(); i++)
	{
		double d = context.getProblemVariablePosition(i) - context.getProblemVariableInitialGuess(i);
		double acc_d = fmax(0.0, fabs(d) / pow(time_step_,2) - acceleration_limits_[i]);
		acc_d *= weight_;
		sum += acc_d * acc_d;
	}

	return sum;
}

/**
 * @brief Constructor for the MultipleGoalsAtOnce class
 */
MultipleGoalsAtOnce::MultipleGoalsAtOnce()
{
	secondary_ = true;
	apply_avoid_joint_limits_goal_ = false;
	apply_minimal_displacement_goal_ = false;
	apply_hard_limits_goal_ = false;
	apply_manipulability_goal_ = false;
	apply_min_velocity_goal_ = false;
	apply_min_acceleration_goal_ = false;
}

void MultipleGoalsAtOnce::applyAvoidJointLimitsGoal(double weight)
{
	w_avoid_joint_limits_ = weight;
	apply_avoid_joint_limits_goal_ = true;
}

void MultipleGoalsAtOnce::applyMinimalDisplacementGoal(double weight)
{
	w_minimum_displacement_ = weight;
	apply_minimal_displacement_goal_ = true;
}

void MultipleGoalsAtOnce::applyHardLimitsGoal(double lower_limit, double upper_limit, int joint_elbow_index, double weight)
{
	w_hard_limits_ = weight;
	apply_hard_limits_goal_ = true;
	lower_limit_ = lower_limit;
	upper_limit_ = upper_limit;
	joint_elbow_index_ = joint_elbow_index;
}

void MultipleGoalsAtOnce::applyManipulabilityGoal(const Eigen::MatrixXd jacobian, double weight)
{
	jacobian_ = jacobian;
	w_manipulability_ = weight;
	apply_manipulability_goal_ = true;
}

void MultipleGoalsAtOnce::applyMinimalVelocityjointCost(double time_step, int joint_index, double weight)
{
	time_step_ = time_step;
	joint_index_ = joint_index;
	w_min_velocity_ = weight;
	apply_min_velocity_goal_ = true;
}

void MultipleGoalsAtOnce::applyMinimalAccelerationCost(const std::vector<double> acceleration_limits, double time_step, double weight)
{
	acceleration_limits_ = acceleration_limits;
	time_step_ = time_step;
	w_min_acceleration_ = weight;
	apply_min_acceleration_goal_ = true;
}

/**
 * @brief Evaluate the cost of the goals and sum them up
 * @param context - the goal context
 * @return the cost of the goals
 */
double MultipleGoalsAtOnce::evaluate(const bio_ik::GoalContext &context) const
{
	double sum = 0.0;

	/*
	// Position and orientation goal in separate cost function (not a secondary one)
	tf2::Vector3 position(target_pose_.translation().x(), target_pose_.translation().y(), target_pose_.translation().z());
	tf2::Vector3 context_position = context.getLinkFrame().getPosition();
	//sum += context.getLinkFrame().getPosition().distance2(getPosition());
	sum += context_position.distance2(position);

	tf2::Quaternion orientation(target_pose_.rotation().x(), target_pose_.rotation().y(), target_pose_.rotation().z(), target_pose_.rotation().w());
	tf2::Quaternion context_orientation = context.getLinkFrame().getOrientation();

	double diff_norm1 = (orientation - context_orientation).length2();
	double diff_norm2 = (orientation + context_orientation).length2();
	double rotation_scale_ = 0.5;
	//sum += fmin((getOrientation() - context.getLinkFrame().getOrientation()).length2(), (getOrientation() + context.getLinkFrame().getOrientation()).length2()) * (rotation_scale_ * rotation_scale_);
	sum += fmin(diff_norm1, diff_norm2) * (rotation_scale_ * rotation_scale_);
	*/

	// minimal displacement goal
	if (apply_minimal_displacement_goal_)
	{
		for (size_t i = 0; i < context.getProblemVariableCount(); i++)
		{
			double d = context.getProblemVariablePosition(i) - context.getProblemVariableInitialGuess(i);
			d *= w_minimum_displacement_;
			sum += d * d;
		}
	}

	// avoid joint limits goal
	if (apply_avoid_joint_limits_goal_)
	{
		auto &info = context.getRobotInfo();
		for (size_t i = 0; i < context.getProblemVariableCount(); i++)
		{
			size_t ivar = context.getProblemVariableIndex(i);
			if (info.getClipMax(ivar) == DBL_MAX)
				continue;
			double d = context.getProblemVariablePosition(i) - (info.getMin(ivar) + info.getMax(ivar)) * 0.5;
			d = fmax(0.0, fabs(d) * 2.0 - info.getSpan(ivar) * 0.5);
			d *= w_avoid_joint_limits_;
			sum += d * d;
		}
	}

	// hard limits goal
	if (apply_hard_limits_goal_)
	{
		double d = context.getProblemVariablePosition(joint_elbow_index_) - (upper_limit_ + lower_limit_) * 0.5;
		d = fmax(0.0, fabs(d) * 2.0 - (upper_limit_ - lower_limit_) * 0.5);
		d *= w_hard_limits_;
		sum += d * d;
	}

	if (apply_manipulability_goal_)
	{
		Eigen::VectorXd singular_values;
		double condition_number = 0.0;
		// double sum = 0.0;
		double min_sv = 0.0;
		bool svd_ = true;

		if (svd_)
		{
			// compute the singular values of the Jacobian
			Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian_, Eigen::ComputeThinU | Eigen::ComputeThinV);
			singular_values = svd.singularValues();

			// Compute the the condition number (The inverse of the condition number is a measure of the manipulability)
			if (singular_values.minCoeff() == 0)
			{
				min_sv = 1e-6;
			}
			else
			{
				min_sv = singular_values.minCoeff();
			}

			condition_number = singular_values.maxCoeff() / min_sv;
			condition_number *= w_manipulability_;
			sum += condition_number * condition_number;
		}
		else
		{
			// Compute the manipulability
			double manipulability = sqrt((jacobian_ * jacobian_.transpose()).determinant());
			if (manipulability == 0)
			{
				manipulability = 1e-6;
			}

			sum += (w_manipulability_ * w_manipulability_) / manipulability;
		}
	}

	// minimal velocity joint goal
	if (apply_min_velocity_goal_)
	{	
		auto &info = context.getRobotInfo();
		double velocity_limit_ = info.getMaxVelocity(joint_index_);
		double d = context.getProblemVariablePosition(joint_index_) - context.getProblemVariableInitialGuess(joint_index_);
		double vel_d = fmax(0.0, fabs(d) / time_step_ - velocity_limit_);
		vel_d *= w_min_velocity_;
		sum += vel_d * vel_d;
	}

	// minimal acceleration joints goal
	if (apply_min_acceleration_goal_)
	{
		
		for (size_t i = 0; i < context.getProblemVariableCount(); i++)
		{
			double d = context.getProblemVariablePosition(i) - context.getProblemVariableInitialGuess(i);
			double acc_d = fmax(0.0, fabs(d) / pow(time_step_,2) - acceleration_limits_[i]);
			acc_d *= w_min_acceleration_;
			sum += acc_d * acc_d;
		}

	}

	return sum;
}

