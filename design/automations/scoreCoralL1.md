# Score Coral In L1

## Subsystems
1. Drivetrain
1. Intake
1. Photonvision

## Steps
1. Determine if coral is present in intake (prevents accidental triggering)
1. Determine position on field
1. Maneuver to reef
1. Align with base of reef
1. Deposit coral (horizontally) 

## Automation 
1. IR Camera: determines if coral is present at beginning of automation, and if it is no longer present at the end of the process
1. Photonvision: determines position on field
1. Pathplanner: Used to move from position to reef automatically
1. ????: Used to align with base of reef
1. Command: command to deposit coral