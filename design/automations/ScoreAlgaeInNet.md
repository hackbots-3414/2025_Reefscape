# Score algae in net

## Subsystems
* Drivetrain
* Camera
* Elevator
* Algae Roller

## Assumptions
* Assume elevator can reach high enough to place algae
* We can hold coral and score in the net.
* We will have to shoot from a small distance (so we don't touch the net).

## Operations
`checkIfHasAlgae`
- Checks if the robot has algae, by using intake current
- Checks if elevator is at height
- Check if we don't have coral
`algaeDropMode`
- Drive to the correct y position, and lock y axis
- Rotate correctly, and lock rotation
- Only allow driver to move in x axis in "turtle mode".
`isReadyToDrop`
- Checks if the location for the robot on the field is correct
`dropAlgae`
- Ejects algae from algae roller with enough speed to go into the net.
- If necessary, we can back up so that we don't
touch the net on the elevator's way down.
- Stow elevator.
- End algae drop mode.