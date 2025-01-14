#  Leave Home 

## Subsystems
* Drivetrain
* Photon Vision

## Assumptions
* We can see an April tag, or we have a predefined starting state that aligns with the robot's real position.
* Chosen auton involves leaving the home.
* The motors aren't blocked

## Operations
`leaveHome`
* Uses the pose estimator of the drivetrain, paired with photonvision, to estimate the robot's position.
* Follows the current auton using PathPlanner.