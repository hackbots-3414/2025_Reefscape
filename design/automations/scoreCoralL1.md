# Score Coral In L1

## Subsystems
1. Drivetrain
1. Intake
1. Camera
1. Elevator

## Assumptions 

1. The robot will align perpendicular to the reef
1. The angle at which the coral is ejected can't be rotated
1. The height at which the coral is ejected can be adjusted

## Operations

`scoreCoralL1`
* Detects nearest scoring postition on the field.
This should be relatively close to the robot's current position, because the driver
should have driven close enough.
* The robot drives to the correct position and orientation.
* The Elevator raises to the correct position as well.
* Until all finish, wait.
* Eject coral

This code will return instantly if there is no coral present.
