# Remove Algae from Reef

## Subsystems

1. Algae Roller
1. Drivetrain
1. Elevator

## Assumptions 

* There are algae in the reef
* There are no obstacles in our path
* We have the mechanism to remove algae
* Algae roller has the ability to pivot forwards or backwards in order to reach algae on the reef
* The roller has a current load sensor to check if an algae is stored within the roller

## Operations 

`removeAlgae`
* The robot approaches specified algae's location according to algae ID given
* The elevator adjusts to the appropriate height to remove the algae from the reef. 
* Intake the Algae
* Return elevator to original height