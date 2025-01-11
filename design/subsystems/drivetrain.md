# Drivetrain

## Assumptions
* Swerve drive used
* 8 motor configuration
* 4 absolute encoders (for steer)
* accurate pose estimation
* obstacles correctly labeled in PathPlanner.
* Uses a gryo that can accurately read the azimuth of the robot.
* We have an error tolerance for position that is within reasonable range, and we do not get stuck in loops due to that.
* Pathplanner's drive to point works.

## Operations
`drive(velocityRef)`
- sets the PID reference for the swerve drive to the given velocity reference.

`driveToCoordinate`
- Calculates a path (via pathplanner's code API) that drives from our current location to
a given point. Then we run that path.