# L2/L3 Coral Score

## Subsystems
* Drivetrain
* Elevator
* Hand

## Assumptions
* Assumptions from each subsystem are still valid.
* The subsystems do as described in their docs.

## Operations

`scoreCoralL2` / `scoreCoralL3`
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