# Climber

## Assumptions
* Robot is physically capable of climbing on deep
climb without breaking any
rules.
* Robot battery has enough amperage to supply power for
the entire climb, and the climb will be balanced after
being disabled. The motors do not need to be actively holding
the climb. However, the motors MAY be in brake mode.
* The climber mechanism has one of the following methods for detecting a finished climb:
    * Limit switch (simplest and most reliable).
    * Encoder counter
    * Range sensor

## Operations
`climbDeep`
* Begins to activate whatever mechanism is responsible for
climbing the deep cage.
* Terminates when previously declared "stopping mechanism" reaches a set limit.