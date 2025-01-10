# Deep climb

## Subsystems
Drivetrain
Climber

## Assumptions
* The closest available climb spot is an open deep climb.
* The other alliance members' robots' climbs would not
interfere with our robot's climb.
* The arm is at its lowest position and is not in the way.
* The cage is not swinging, OR the mechanical design is
compatible with such an event.

## Operations
`deepClimb`
* The robot (in its ready state) finds and drives towards
the nearest climb spot.
* The climb mechanism will be started with the climber's `climbDeep` operation.