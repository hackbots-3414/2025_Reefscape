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

## Algae Intake

### Assumptions
* Physically capable of intaking algae

### Operations
`Intake`

`Eject` - Regardless of whether intake is the same as output, must be capable of eject to prevent a jam

`AutoIntake` - Moves forward when camera detects proper alignment on own

`Hold` - Touch it, own it. Keep holding gamepiece once intaked/intook/intaken/took in/taken in

## Algae Output

### Assumptions
* Physically capable of shooting algae into basket
* Physically capable of depositing algae into processor

### Operations
`Shoot`

## Coral Intake

### Assumptions
* Physically capable of intaking coral

### Operations
`Intake`

`Eject` - Regardless of whether intake is the same as output, must be capable of eject to prevent a jam

`AutoIntake` - Moves forward when camera detects proper alignment on own

`Hold` - Touch it, own it. Keep holding gamepiece once intaked/intook/intaken/took in/taken in

## Coral Output

### Assumptions
* Physically capable of depositing coral on all levels

### Operations
`Out` - Eject coral into goal levels

## Camera

## Assumptions
* Camera capable of taking good enough pictures to identify coral, AprilTags

### Operations
`FindCoral`

`FindCoralPos`

`AprilTagPos`
