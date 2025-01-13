# Score Next Best Coral

## Subsystems
* Drivetrain
* Camera
* Output

## Assumptions
* We can see an April tag, or we can semi-correctly estimate the position on the field.
* The robot currently has a ready coral.
* There is a next best coral
* Our "database" of placed coral accurately reflects the current state of the reef.

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