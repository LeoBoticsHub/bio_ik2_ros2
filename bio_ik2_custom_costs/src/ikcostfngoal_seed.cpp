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
	  seed_state_(seed_state) {
	// set the weight of the goal
	setWeight(weight);
}

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

MinimalDisplacementGoalSeed::MinimalDisplacementGoalSeed(const std::vector<double> &seed_state,
														 double weight, bool secondary)
	: seed_state_(seed_state) {
	weight_ = weight;
	secondary_ = secondary;
}

double MinimalDisplacementGoalSeed::evaluate(const GoalContext &context) const {
	double sum = 0.0;
	for (size_t i = 0; i < context.getProblemVariableCount(); i++) {
		double d = context.getProblemVariablePosition(i) - seed_state_[i];
		sum += d * d;
	}
	return sum * weight_;
}

ConfigureElbowGoal::ConfigureElbowGoal(const int joint_elbow_index, double lower_limit, 
double upper_limit, double weight)
: joint_elbow_index_(joint_elbow_index), 
lower_limit_(lower_limit), 
upper_limit_(upper_limit) {
weight_ = weight;
}

double ConfigureElbowGoal::evaluate(const GoalContext &context) const {
double sum = 0.0;
double d = context.getProblemVariablePosition(joint_elbow_index_) - (upper_limit_ + lower_limit_) * 0.5;
d = fmax(0.0, fabs(d) * 2.0 - (upper_limit_ - lower_limit_) * 0.5);
d *= weight_;
sum += d * d;

return sum;
}

