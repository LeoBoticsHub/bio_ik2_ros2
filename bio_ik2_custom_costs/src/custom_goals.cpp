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
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "custom_goals.hpp"
#include <algorithm>
#include <cmath>

using namespace bio_ik;

// ============================================================
// IKCostFnGoalSeed
// ============================================================

IKCostFnGoalSeed::IKCostFnGoalSeed(const geometry_msgs::msg::Pose &pose,
								   const kinematics::KinematicsBase::IKCostFn &function,
								   const moveit::core::RobotModelConstPtr &robot_model,
								   const std::vector<double> &seed_state,
								   double weight)
	: Goal(), pose_(pose), function_(function), robot_model_(robot_model), seed_state_(seed_state) {
	setWeight(weight);
}

double IKCostFnGoalSeed::evaluate(const GoalContext &context) const {
	moveit::core::RobotState robot_state(robot_model_);
	auto jmg = context.getJointModelGroup();

	std::vector<double> sol_positions(context.getProblemVariableCount());
	for (size_t i = 0; i < context.getProblemVariableCount(); ++i)
		sol_positions[i] = context.getProblemVariablePosition(i);

	robot_state.setJointGroupPositions(&jmg, sol_positions);
	robot_state.update();
	return function_(pose_, robot_state, &jmg, seed_state_);
}

// ============================================================
// MinimalDisplacementGoalSeed
// ============================================================

MinimalDisplacementGoalSeed::MinimalDisplacementGoalSeed(const std::vector<double> &seed_state,
														 double weight, bool secondary)
	: seed_state_(seed_state) {
	weight_ = weight;
	secondary_ = secondary;
}

double MinimalDisplacementGoalSeed::evaluate(const GoalContext &context) const {
	double sum = 0.0;
	for (size_t i = 0; i < context.getProblemVariableCount(); i++) {
		const double d = context.getProblemVariablePosition(i) - seed_state_[i];
		sum += d * d;
	}
	return sum * weight_;
}

// ============================================================
// HardJointLimitsGoal
// ============================================================

HardJointLimitsGoal::HardJointLimitsGoal(int joint_index, double lower_limit,
										 double upper_limit, double weight)
	: lower_limit_(lower_limit), upper_limit_(upper_limit), joint_index_(joint_index) {
	secondary_ = true;
	weight_ = weight;
}

double HardJointLimitsGoal::evaluate(const GoalContext &context) const {
	double d = context.getProblemVariablePosition(joint_index_) -
			   (upper_limit_ + lower_limit_) * 0.5;
	d = fmax(0.0, fabs(d) * 2.0 - (upper_limit_ - lower_limit_) * 0.5);
	return d * d * weight_;
}

// ============================================================
// SlidekitFollowXGoal
// ============================================================

SlidekitFollowXGoal::SlidekitFollowXGoal(int joint_index, double point_x,
										 double offset, double clamp_min, double clamp_max,
										 double weight)
	: joint_index_(joint_index), point_x_(point_x), offset_(offset),
	  clamp_min_(clamp_min), clamp_max_(clamp_max) {
	secondary_ = true;
	weight_ = weight;
}

double SlidekitFollowXGoal::evaluate(const GoalContext &context) const {
	const double px = std::clamp(point_x_, clamp_min_, clamp_max_);
	const double q_slide = context.getProblemVariablePosition(joint_index_);
	const double d = px - q_slide - offset_;
	return d * d * weight_;
}

// ============================================================
// SlidekitConstantDistanceGoal
// ============================================================

SlidekitConstantDistanceGoal::SlidekitConstantDistanceGoal(int joint_index,
														   double point_x, double point_y,
														   double offset_x, double d_target,
														   double weight)
	: joint_index_(joint_index), point_x_(point_x), point_y_(point_y),
	  offset_x_(offset_x), d_target_(d_target) {
	secondary_ = true;
	weight_ = weight;
}

double SlidekitConstantDistanceGoal::evaluate(const GoalContext &context) const {
	const double q_slide = context.getProblemVariablePosition(joint_index_);
	const double dx = point_x_ - q_slide - offset_x_;
	const double actual_dist = std::sqrt(dx * dx + point_y_ * point_y_);
	const double err = actual_dist - d_target_;
	return err * err * weight_;
}

// ============================================================
// MultipleGoalsAtOnce
// ============================================================

MultipleGoalsAtOnce::MultipleGoalsAtOnce() {
	secondary_ = true;
	weight_ = 1.0;
}

void MultipleGoalsAtOnce::applyMinimalDisplacementGoal(double weight) {
	w_minimum_displacement_ = weight;
	apply_minimal_displacement_goal_ = true;
}

void MultipleGoalsAtOnce::applyHardLimitsGoal(double lower_limit, double upper_limit,
											  int joint_index, double weight) {
	hard_limit_entries_.push_back({lower_limit, upper_limit, joint_index, weight});
	apply_hard_limits_goal_ = true;
}

void MultipleGoalsAtOnce::applySlidekitFollowXGoal(int joint_index, double point_x,
												   double offset, double clamp_min,
												   double clamp_max, double weight) {
	apply_slidekit_follow_x_goal_ = true;
	slidekit_follow_x_joint_index_ = joint_index;
	follow_x_point_x_ = point_x;
	follow_x_offset_ = offset;
	follow_x_clamp_min_ = clamp_min;
	follow_x_clamp_max_ = clamp_max;
	w_slidekit_follow_x_ = weight;
}

void MultipleGoalsAtOnce::applySlidekitConstantDistanceGoal(int joint_index,
															double point_x, double point_y,
															double offset_x, double d_target,
															double weight) {
	apply_slidekit_constant_distance_goal_ = true;
	slidekit_cd_joint_index_ = joint_index;
	cd_point_x_ = point_x;
	cd_point_y_ = point_y;
	cd_offset_x_ = offset_x;
	cd_d_target_ = d_target;
	w_slidekit_constant_distance_ = weight;
}

double MultipleGoalsAtOnce::evaluate(const bio_ik::GoalContext &context) const {
	double sum = 0.0;

	// ---- minimal displacement ----------------------------------------
	if (apply_minimal_displacement_goal_) {
		for (size_t i = 0; i < context.getProblemVariableCount(); i++) {
			const double d = context.getProblemVariablePosition(i) -
							 context.getProblemVariableInitialGuess(i);
			sum += d * d * w_minimum_displacement_;
		}
	}

	// ---- hard joint limits -------------------------------------------
	if (apply_hard_limits_goal_) {
		for (const auto &e : hard_limit_entries_) {
			double d = context.getProblemVariablePosition(e.joint_index) -
					   (e.upper_limit + e.lower_limit) * 0.5;
			d = fmax(0.0, fabs(d) * 2.0 - (e.upper_limit - e.lower_limit) * 0.5);
			sum += d * d * e.weight;
		}
	}

	// ---- slidekit follow-X -------------------------------------------
	if (apply_slidekit_follow_x_goal_ && w_slidekit_follow_x_ > 0.0) {
		const double px = std::clamp(follow_x_point_x_, follow_x_clamp_min_, follow_x_clamp_max_);
		const double q = context.getProblemVariablePosition(slidekit_follow_x_joint_index_);
		const double d = px - q - follow_x_offset_;
		sum += d * d * w_slidekit_follow_x_;
	}

	// ---- slidekit constant distance ----------------------------------
	if (apply_slidekit_constant_distance_goal_ && w_slidekit_constant_distance_ > 0.0) {
		const double q = context.getProblemVariablePosition(slidekit_cd_joint_index_);
		const double dx = cd_point_x_ - q - cd_offset_x_;
		const double actual_dist = std::sqrt(dx * dx + cd_point_y_ * cd_point_y_);
		const double err = actual_dist - cd_d_target_;
		sum += err * err * w_slidekit_constant_distance_;
	}

	return sum;
}