# Subsystems

## Drivetrain

### Assumptions
* Swerve drive used
* 8 motor configuration
* 4 absolute encoders (for steer)

### Operations
`drive`
* Move around based on input distance
* Takes parameter for distance

`driveToCoordinate`
* Should this avoid obstacles?

## Algae Intake

### Assumptions
* Physically capable of intaking algae

### Operations
`AlgaeIntake` - Get algae into the robot

`Eject` - Gets algae out of intake.

`AutoIntake` - Moves forward when camera detects proper alignment on own, then uses Intake command to get the algae into the robot

`Hold` - Touch it, own it. Keep holding gamepiece once intaked/intook/intaken/took in/taken in (Don't know the word)

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
`CoralIntake` - Get coral into the robot

`Eject` - Regardless of whether intake is the same as output, must be capable of eject to prevent a jam. Gets coral out of intake.

`AutoIntake` - Moves forward when camera detects proper alignment on own, then uses CoralIntake to get coral into the robot

`Hold` - Touch it, own it. Keep holding gamepiece once intaked/intook/intaken/took in/taken in

## Coral Output

### Assumptions
* Physically capable of depositing coral on all levels

### Operations
`Out(goalNum)` - Eject coral into goal level. Takes parameter of desired level.

## Camera

### Assumptions
* Camera capable of taking good enough pictures to identify coral, AprilTags, algae

* PhotonVision is reliable enough to consistently provide an accurate pose
estimation.

* We cannot update our location when moving at sufficiently high speeds.

### Operations
`FindCoral` - Locates PVC gamepeice

`FindAlgae` - Locates teal rubber ball gamepeice

`FindCoralPos` - Checks what PVC gamepeices have already been scored on reef

`AprilTagPos` - Uses data generated from AprilTags in order to ascertain robot's location on the field
