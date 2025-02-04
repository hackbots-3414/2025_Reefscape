# Score Next Best Coral

## Subsystems
* Drivetrain
* Camera
* Coral Rollers

## Assumptions
* We can see an April tag, or we can correctly estimate the position on the field.
* The robot currently has a ready coral.
* There is a next best coral

## Operations
`scoreNextBestCoral`
* Look up in database the "next best" coral:
    * prioritize coral closer to L4
    * prioritize coral closer to robot
* Move to the corresponding position for that coral.
As this happens, we should raise the elevator to the correct position.
This will block until both have finished.
* Release coral, score it.
* Mark that coral spot as "scored" in our database.