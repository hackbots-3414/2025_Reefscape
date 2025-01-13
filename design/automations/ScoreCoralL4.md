# Score Coral L4
* Obtain coral
* Get to reef
* Line up with 4th level pole
* Rotate coral to line up with 4th level
* Place coral on reef
* Release coral
* Back away
## Subsystems:
* Drivetrain
* Intake
* Output
* Camera
* Elevator

## Assumptions: 

* The angle at which the coral is ejected can't be rotated
* The height at which the coral is ejected can be adjusted

## Operations 

`scoreCoralL4`
* Detects nearest scoring postition on the field.
This should be relatively close to the robot's current position, because the driver
should have driven close enough.
The robot drives to the correct position and orientation.
The Elevator raises to the correct position as well.
Then, rotate wrist to the correct angle (get exact number from CAD)
As we do that, ensure that the hand is rotated so the coral is perpendicular.
Until all finish, wait.
Then, open hand.

This code will return instantly if there is no coral present.reef
