# Score Coral In L1

## Subsystems
1. Drivetrain
1. Intake
1. Camera
1. Elevator

## Assumptions 

1. The angle at which the coral is ejected can't be rotated
1. The height at which the coral is ejected can be adjusted

## Operations

`scoreCoralL1`
* Detects nearest scoring postition on the field.
This should be relatively close to the robot's current position, because the driver
should have driven close enough.
The robot drives to the correct position and orientation.
The Elevator raises to the correct position as well.
Then, rotate wrist to the correct angle (get exact number from CAD)
As we do that, ensure that the hand is rotated so the coral is perpendicular.
Until all finish, wait.
Then, open hand.

This code will return instantly if there is no coral present.
