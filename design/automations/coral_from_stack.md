# Coral From Stack (coral_from_stack)

## Subsystems
* Arm
* Drivetrain
* Hand

## Assumptions
* "hand" has ability to grab vertical coral
* "hand" can rotate in whatever orientation
* Arm can reach low enough to the stack
* Hand can tilt up and down.

## Operations
`pickUpCoralFromStack(stackId)`
 * Drives to the stack corresponding to that ID
 * Lowers arm as the drive happens to the stack position
 * Sets the arm's wrist to parallel to the horizontal.
 * Once the position is reached, then the hand closes and grabs a coral.


