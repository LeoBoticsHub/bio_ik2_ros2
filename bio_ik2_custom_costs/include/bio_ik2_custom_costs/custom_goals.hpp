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

#ifndef CUSTOM_GOALS_HPP
#define CUSTOM_GOALS_HPP

#include "bio_ik/goal.h"
#include "bio_ik/robot_info.h"

#include <vector>

namespace bio_ik {

/**
 * @brief Wraps an arbitrary MoveIt IKCostFn as a BioIK Goal.
 *        Useful for injecting standard MoveIt cost functions into the BioIK pipeline.
 */
class IKCostFnGoalSeed : public Goal {
	const geometry_msgs::msg::Pose pose_;
	const kinematics::KinematicsBase::IKCostFn function_;
	const moveit::core::RobotModelConstPtr robot_model_;
	const std::vector<double> seed_state_;

public:
	IKCostFnGoalSeed(const geometry_msgs::msg::Pose &pose,
					 const kinematics::KinematicsBase::IKCostFn &function,
					 const moveit::core::RobotModelConstPtr &robot_model,
					 const std::vector<double> &seed_state,
					 double weight = 1.0);

	double evaluate(const GoalContext &context) const override;
};

/**
 * @brief Minimises the sum of squared joint displacements from a fixed seed state.
 */
class MinimalDisplacementGoalSeed : public Goal {
private:
	const std::vector<double> seed_state_;

public:
	MinimalDisplacementGoalSeed(const std::vector<double> &seed_state,
								double weight = 1.0,
								bool secondary = true);

	double evaluate(const GoalContext &context) const override;
};

/**
 * @brief Keeps a joint inside a soft virtual range centred on
 *        (upper_limit + lower_limit) / 2.
 *        Cost grows quadratically once the joint leaves the inner half of the range.
 */
class HardJointLimitsGoal : public Goal {
private:
	const double lower_limit_;
	const double upper_limit_;
	const int joint_index_;

public:
	/**
	 * @param joint_index  Index inside the IK problem variables.
	 * @param lower_limit  Lower bound of the allowed range [rad].
	 * @param upper_limit  Upper bound of the allowed range [rad].
	 * @param weight       Goal weight (default 1.0).
	 */
	HardJointLimitsGoal(int joint_index, double lower_limit, double upper_limit,
						double weight = 1.0);

	double evaluate(const GoalContext &context) const override;
};

/**
 * @brief Pushes the slidekit joint to minimise the 1-D error on the slide axis:
 *
 *   cost = ( clamp(point_x, clamp_min, clamp_max) - q_slidekit - offset )^2 * weight
 *
 * point_x must be refreshed at every IK call by creating a new instance.
 */
class SlidekitFollowXGoal : public Goal {
private:
	const int joint_index_;
	const double point_x_;
	const double offset_;
	const double clamp_min_;
	const double clamp_max_;

public:
	/**
	 * @param joint_index  Index of the slidekit joint (usually 0).
	 * @param point_x      X-coordinate of the current IK target.
	 * @param offset       Fixed x-offset between slidekit flange and arm centre [m].
	 * @param clamp_min    Minimum allowed reference x [m].
	 * @param clamp_max    Maximum allowed reference x [m].
	 * @param weight       Goal weight (default 1.0).
	 */
	SlidekitFollowXGoal(int joint_index, double point_x,
						double offset, double clamp_min, double clamp_max,
						double weight = 1.0);

	double evaluate(const GoalContext &context) const override;
};

/**
 * @brief Maintains a constant 2-D distance between the slidekit flange and the
 *        end-effector target, measured in the XY plane of the slide frame:
 *
 *   actual_dist = sqrt( (point_x - q_slidekit - offset_x)^2 + point_y^2 )
 *   cost        = ( actual_dist - d_target )^2 * weight
 *
 * Set weight = 0 to disable without removing the goal from the pipeline.
 * Both point_x and point_y must be refreshed at every IK call by creating a new instance.
 */
class SlidekitConstantDistanceGoal : public Goal {
private:
	const int joint_index_;
	const double point_x_;
	const double point_y_;
	const double offset_x_;
	const double d_target_;

public:
	/**
	 * @param joint_index  Index of the slidekit joint (usually 0).
	 * @param point_x      X-coordinate of the current IK target.
	 * @param point_y      Y-coordinate of the current IK target.
	 * @param offset_x     Fixed x-offset between slidekit flange and arm centre [m].
	 * @param d_target     Desired constant distance (flange → EE) [m].
	 * @param weight       Goal weight (default 1.0).  Set to 0 to disable.
	 */
	SlidekitConstantDistanceGoal(int joint_index,
								 double point_x, double point_y,
								 double offset_x, double d_target,
								 double weight = 1.0);

	double evaluate(const GoalContext &context) const override;
};

/**
 * @brief Combines multiple secondary goals into a single BioIK Goal object.
 *
 * Sub-goals enabled via apply*() methods:
 *   - Minimal displacement from initial guess.
 *   - Hard virtual joint limits for one or more joints.
 *   - Slidekit follow-X (1-D error on the slide axis).
 *   - Slidekit constant 2-D distance (XY-plane distance from flange to EE target).
 */
class MultipleGoalsAtOnce : public Goal {
private:
	// ---- minimal displacement ----------------------------------------
	bool apply_minimal_displacement_goal_ = false;
	double w_minimum_displacement_ = 0.0;

	// ---- hard joint limits (one entry per joint) ---------------------
	bool apply_hard_limits_goal_ = false;
	struct HardLimitEntry {
		double lower_limit;
		double upper_limit;
		int joint_index;
		double weight;
	};
	std::vector<HardLimitEntry> hard_limit_entries_;

	// ---- slidekit follow-X -------------------------------------------
	bool apply_slidekit_follow_x_goal_ = false;
	double w_slidekit_follow_x_ = 0.0;
	int slidekit_follow_x_joint_index_ = 0;
	double follow_x_point_x_ = 0.0;
	double follow_x_offset_ = 0.0;
	double follow_x_clamp_min_ = 0.0;
	double follow_x_clamp_max_ = 0.0;

	// ---- slidekit constant distance ----------------------------------
	bool apply_slidekit_constant_distance_goal_ = false;
	double w_slidekit_constant_distance_ = 0.0;
	int slidekit_cd_joint_index_ = 0;
	double cd_point_x_ = 0.0;
	double cd_point_y_ = 0.0;
	double cd_offset_x_ = 0.0;
	double cd_d_target_ = 0.0;

public:
	MultipleGoalsAtOnce();

	/** Enable minimal displacement from the initial guess. */
	void applyMinimalDisplacementGoal(double weight = 1.0);

	/**
	 * @brief Add a hard-limits entry for one joint.
	 *        Call once per joint; entries accumulate (not overwritten on repeated calls).
	 */
	void applyHardLimitsGoal(double lower_limit, double upper_limit,
							 int joint_index, double weight = 1.0);

	/**
	 * @brief Enable/update the slidekit follow-X goal.
	 *        Must be called every IK solve so that point_x reflects the current target.
	 */
	void applySlidekitFollowXGoal(int joint_index, double point_x,
								  double offset, double clamp_min, double clamp_max,
								  double weight = 1.0);

	/**
	 * @brief Enable/update the slidekit constant-distance goal.
	 *        Must be called every IK solve so that point_x / point_y reflect the current target.
	 *        Set weight = 0 to disable without removing the goal from the pipeline.
	 */
	void applySlidekitConstantDistanceGoal(int joint_index,
										   double point_x, double point_y,
										   double offset_x, double d_target,
										   double weight = 1.0);

	double evaluate(const bio_ik::GoalContext &context) const override;
};

} // namespace bio_ik

#endif // CUSTOM_GOALS_HPP