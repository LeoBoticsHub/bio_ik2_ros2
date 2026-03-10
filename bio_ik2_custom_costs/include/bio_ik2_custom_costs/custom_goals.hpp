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
 * @brief Combines multiple secondary goals into a single BioIK Goal object.
 *
 * Sub-goals enabled via apply*() methods:
 *   - Minimal displacement from initial guess.
 *   - Soft virtual joint limits for one or more joints.
 *   - Slidekit follow-X (1-D error on the slide axis).
 *   - Slidekit constant 2-D distance (XY-plane distance from slidekit-flange to EE target).
 */
class MultipleGoalsAtOnce : public Goal {
private:
	// ---- minimal displacement ----------------------------------------
	bool apply_minimal_displacement_goal_ = false;
	double w_minimum_displacement_ = 0.0;

	// ---- soft joint limits (one entry per joint) ---------------------
	bool apply_soft_limits_goal_ = false;
	struct SoftLimitEntry {
		double lower_limit;
		double upper_limit;
		int joint_index;
		double weight;
	};
	std::vector<SoftLimitEntry> soft_limit_entries_;

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
	 * @brief Add a soft-limits entry for one joint.
	 *        Call once per joint; entries accumulate (not overwritten on repeated calls).
	 */
	void applySoftLimitsGoal(double lower_limit, double upper_limit,
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