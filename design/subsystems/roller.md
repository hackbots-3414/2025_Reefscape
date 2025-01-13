# Roller

## Assumptions
* can grab or release a coral
* We can measure the voltage being applied to the motor, and the roller's speed
* IR sensors can be used to detect the presence of a coral in the roller.

## Operations
`eject(speed)`
- Rotates rollers at the given speed that softly ejects the coral.

`stop`
- Stops the rollers

`hasCoral`
- Using IR sensors, check if there is coral in rollers.

- We can use some logic from 2023 to detect if we are pushing against a gamepiece.
If so, we can apply a lower grip voltage.
This could also be used to detect if we have a gamepiece.

