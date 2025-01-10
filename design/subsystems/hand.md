# Hand

## Assumptions
* can be rotated continuously
* can grab or release a coral
* We can measure the voltage being applied to the motor, and the hand's speed
* we can accurately predict the presence of a gamepiece via comparing voltage
and the speed at which the hand is closing (like in 2023 Charged Up!)

## Operations
`close`
- Closes the hand

`open`
- Opens the hand

`setAngle(angle)`
- Rotates the hand to the desired angle.

`getStatus`
- Returns the status of the hand, as one of the following enum variants:

```
Open
ClosedEmpty
ClosedFull
```

Any in-between states could be considered `Open`, I suppose.

- We can use some logic from 2023 to detect if we are pushing against a gamepiece.
If so, we can apply a lower grip voltage.
This could also be used to detect if we have a gamepiece.

