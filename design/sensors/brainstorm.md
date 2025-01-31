### Elevator:
- Lidar distance sensor to measure elevator height
- Lift encoders - absolute encoders - Ask Chris
    - Will they reset to zero on robot boot?
    - Check with Chris - He’s been looking into absolute encoders’ accuracy.

### Algae:
- Measure current on motor to tell if algae is present
- Touch sensor - See if algae is pressing against it (Only if there’s something that the algae will repeatedly end up touching) Ask Design

### Coral:
- IR sensor pointing straight down to see if coral is present
    - Is one IR sensor enough? Consider Crescendo bot. Ask ????????
- Measure motor’s current output
    - With belts, would current change enough to be readable? Ask Mechanical/Electrical

### Donut Climb: Ask Design about climb tactics
- Driver cam for alignment (So we can see inside donut climber)
    - Orientation?
    - On the bottom of the coral funnel?
        - How do we adjust for the angle of the robot? (Robot moves field relative, not camera relative)
            - Robot relative navigation for navigation of cage - Activated by driver
                - Could be confusing. Ask experienced driver

### Odometry:
- Can we see tags under barge, or is it solely on ticks? Ask Nolan/Look at field
- Does the cage spin too much for this? Check with Mechanical

### Funnel Climb:
- Motor current measuring
- Ensure ‘grip’ is strong enough to support robot
- Is there a motor used? Ask Design

### Navigation/Coral Scoring:
- Odometry
- AprilTag location 
    - Blind spots
        - Is there anywhere where we can’t see a single tag? Ask Nolan.
        - Poor confidence spots
    - Is there anywhere where any camera usage fails to accurately see any ArilTags? Ask Nolan.
- Accuracy of tick to distance measurements on drivetrain? Ask ???????