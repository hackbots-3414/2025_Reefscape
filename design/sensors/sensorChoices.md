### Key:
- <span style="color:magenta">Already decided by mechanical/electrical/CAD/design in general</span>
- <span style="color:red">Must be asked for in order to get</span>
- <span style="color:green">No more action needed</span>
- <span style="color:orange">Must wait for outside testing to be done</span>
- <span style="color:yellow">Unclear/uncertain</span>

### Elevator:
- <span style="color:magenta">CANcoder</span> & <span style="color:red">Lidar sensor</span> (facing up to set the elevator to zero on boot) - *necessary because the CANcoders will loop and have multiple zeros* - and <span style="color:red">Lower and Upper Limit Switch</span> (Make sure robot does not tear elevator apart) 

### Algae:
- <span style="color:green">Motor Current</span>

### Coral:
- <span style="color:red">Several IR sensors</span> in funnel or in rollers (<span style="color:yellow">Ask about path coral is going to be taking/where the coral is going to be stored while in transit.</span>) Used for telling where the coral is in the rollers. Two IR. One to check to make sure it isn't sticking out the front, one to check if it's sitting in the middle.
    - IR Sensors are facing down, not sideways. This is due to little available space on the sides.
- <span style="color:green">Motor Current</span> to see if we have grip on coral.

### Climber:
- <span style="color:green">Motor Current</span>
- <span style="color:yellow">Switch</span> to confirm contact
    - What sort of switch? Limit Switch.
    - Design doesn't really like the idea of mounting a switch into the hook thing (Complicated mounting/wiring), but it would be very useful for programming to be able to tell if contact has been made because that hook doesn't look like it'll have good driver/operator visibility especially from the station farthest away from the Barge. Navigation:
- AprilTags when possible - <span style="color:orange">Camera sensors</span>. Nolan says eight so far. Confirm and ask him for more details like position of cameras after he finishes testing.
- <span style="color:green">Drivetrain CANCoders</span> when not possible to detect tags accurately