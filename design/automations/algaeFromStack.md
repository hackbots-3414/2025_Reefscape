# Algae From Stack

## Subsystems
* Drivetrain
* Intake
* Photonvision

## Assumptions
* There is a stack that has an available algae.

## Operations
`getAlgaeFromStack(stackId)`
* Drive to correct intaking position for the stack that corresponds to the stack ID.
* Set elevator to the correct height (determined experimentally).
* Run rollers for the intake
* Use IR sensor to determine when an algae is obtained.
* Stop the intake rollers once we have obtained an algae.