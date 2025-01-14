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
* Coral Intake
* Rollers
* Camera
* Elevator

## Assumptions: 

* The height at which the coral is ejected can be adjusted
* Ejection angle of the coral is fixed
* Elevator subsystem should implement turtle mode when elevator is raised

## Operations 

`scoreCoralL4`
* Detects nearest scoring postition on the field.
This should be relatively close to the robot's current position, because the driver
should have driven close enough.
* The robot drives to the correct position and orientation.
* The Elevator raises to the correct position as well.
* Until all finish, wait.
* Eject coral from rollers
* Lower elevator

This code will return instantly if there is no coral present.reef
