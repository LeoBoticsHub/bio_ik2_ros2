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

void MultipleGoalsAtOnce::applySoftLimitsGoal(double lower_limit, double upper_limit,
											  int joint_index, double weight) {
	soft_limit_entries_.push_back({lower_limit, upper_limit, joint_index, weight});
	apply_soft_limits_goal_ = true;
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

	// ---- soft joint limits -------------------------------------------
	if (apply_soft_limits_goal_) {
		for (const auto &e : soft_limit_entries_) {
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