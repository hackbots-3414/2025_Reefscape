### Throw Algae
* Obtain algae
* Move algae from intake to output if necessary
* Aim for basket
* Shoot
## Subsystems
* Drivetrain
* Camera or encoders
* Algae Intake
* Algae Output
## Automations
* IntakeAlgae
* ShootAlgae
* AlignTargetBasket
* FindPos
# Score algae in net

## Subsystems
Drivetrain
Vision
Elevator
Algae

## Assumptions
- Assume elevator can reach high enough to place algae
- 

## Operations
`checkIfHasAlgae`
- Checks if the robot has algae, by using intake current
`algaeDropMode`
- Drive to the correct y position, and lock y axis
- Rotate correctly, and lock rotation
- Only allow driver to move in x axis
`isReadyToDrop`
- Checks if the location for the robot on the field is correct
- `checkIfHasAlgae`
- Checks if elevator is at height
- Check if we don't have coral
`dropAlgae`