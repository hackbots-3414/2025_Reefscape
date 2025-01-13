# Deep climb

## Subsystems
Drivetrain
Climber

## Assumptions
* The closest available climb spot is an open deep climb.
* The other alliance members' robots' climbs would not
interfere with our robot's climb.
* The Elevator is at its lowest position and is not in the way.
* The cage is not swinging, OR the mechanical design is
compatible with such an event.

## Operations
`deepClimb`
* The robot (in its ready state) finds and drives towards
the nearest climb spot.
* The driver places the robot near one of the three cage locations
* Using the robot's position, it "snaps" to the closest cage location
* The climb mechanism will be started with the climber's `climbDeep` operation.
`checkIfReadyToClimbDeep`
* Checks if the robot is close enough to the cage location to start `deepClimb`
* Checks if the robot is within the last ~15 seconds of match
* Check if intake is flipped down

