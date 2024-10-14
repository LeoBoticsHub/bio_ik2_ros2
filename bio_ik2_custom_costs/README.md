# Custom goals

This package contains custom goals (cost functions) for Bio IK2.

## Contents

The following cost functions are included in this package:

- **IKCostFnGoalSeed**: custom cost function passed as a lambda function to the Bio IK2 solver, using a pre-defined seed state for the optimization.
- **MinimalDisplacementGoalSeed**: minimizes the displacement of the end effector from the given seed state.
- **ConfigureElbowGoal**: configures the elbow of the robot manipulator to be either straight or bent.
- **MaxManipulabilityGoal**: maximizes the manipulability of the robot manipulator.
- **MinimalVelocityJointGoal**: minimizes the velocity of the joints of the robot manipulator, from the solver's initial guess
- **MinimalAccelerationGoal**: minimizes the acceleration of the joints of the robot manipulator, from the solver's initial guess
- **MultipleGoalsAtOnce**: combines multiple goals into a single cost function, with different weights for each goal. It is often more stable than adding multiple goals separately.