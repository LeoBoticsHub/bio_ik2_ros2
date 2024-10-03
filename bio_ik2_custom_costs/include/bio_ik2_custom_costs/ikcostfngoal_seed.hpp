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

#include "bio_ik/goal.h"
#include "bio_ik/robot_info.h"

namespace bio_ik {

// class defining a new Goal supported by BioIkKinematicsQueryOptions
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

class MinimalDisplacementGoalSeed : public Goal {
private:
	const std::vector<double> seed_state_;

public:
	MinimalDisplacementGoalSeed(const std::vector<double>& seed_state, double weight = 1.0, bool secondary = true);
	double evaluate(const GoalContext &context) const;
};


// ConfigureElbowGoal tries to keep the elbow joint in the center half of the specified joint limits
// If the elbow joint is in the center of the specified joint limits ((upper_limit_ + lower_limit_) * 0.5) , the cost is 0
// If the elbow joint is at the upper or lower limit, the cost is proportional to the distance from the center
// The result is double and reduced by the half-span of the joint's range ((upper_limit_ - lower_limit_) * 0.5). This operation centers the deviation around zero.
class ConfigureElbowGoal : public Goal {
private:
	const double lower_limit_;
	const double upper_limit_;
	const int joint_elbow_index_;

public:
	ConfigureElbowGoal(const int joint_elbow_index, const double lower_limit, const double upper_limit, double weight = 1.0);
	double evaluate(const GoalContext &context) const;
};

// MaxManipulabilityGoal tries to maximize the manipulability 
// if svd == true, the singular value decomposition is used to compute the manipulability:
// Compute the the condition number (The inverse of the condition number is a measure of the manipulability)
// As this cost will be minimized and we want to maximize manipulability, we return the condition number*weight
// A high inverse of the condition number means a high manipulability -> a low condition number implies a high manipulability
// If svd == false, the manipulability is computed as the square root of the determinant of the Jacobian times its transpose and its inverse is minimized
class MaxManipulabilityGoal : public Goal {
private:
	const Eigen::MatrixXd jacobian_;
	 bool svd_;

public:
	MaxManipulabilityGoal(const Eigen::MatrixXd jacobian, bool svd, double weight = 1.0);
	double evaluate(const GoalContext &context) const;
};

} // namespace bio_ik
