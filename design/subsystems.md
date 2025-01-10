# Subsystems

## Drivetrain

### Assumptions
* Swerve drive used
* 8 motor configuration
* 4 absolute encoders (for steer)

### Operations
`drive`

`driveToCoordinate`
* Should this avoid obstacles?

## Intake

### Assumptions
* Physically capable of intaking coral

### Operations
`Intake`

`Eject`

`AutoIntake` - Moves forward when camera detects proper alignment on own

`Hold` - Touch it, own it. Keep holding gamepiece once intaked/intook/intaken/took in/taken in

## Camera

## Assumptions
* Camera capable of taking good enough pictures to identify coral, AprilTags

### Operations
`FindCoral`

`AprilTagPos`
