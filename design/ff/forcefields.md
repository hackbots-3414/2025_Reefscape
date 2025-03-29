# Force fields

A __force field__ is a location on the field that the robot should not be able
to go to. The following explains the algorithm of the force field, for those
interested.

## General process

Applying a force field requires control over the robot's drivetrain during the
teleoperated portion of the match. So, it is natural that this code should go in
the default `TeleopCommand` class. This is the command that is used to drive the
drivetrain in response to the driver's input.

To simplify this process, a new class (perhaps call it `DriverAssist`) can be created, and it will be responsible for altering the driver's input (requested velocity) and outputting a new velocity to be used instead. This is how the general driver assist works. Given some driver input *c*, the system produces an output *r* that better helps the driver.

In this case, the force field logic uses velocity vectors as inputs and outputs. The angular velocity of the robot is unchanged throughout this process and will therefore be excluded from further discussion. The general process for a force field's adjustment on a driver input follows these general steps:

1. Determine the distance from the robot's current location on the field and the __antitarget__ - the position the robot is NOT allowed to go.
1. Doing a little thinking in reverse, assume that the robot ends exactly at the antitarget. Running time in reverse and assuming that we travel at some known acceleration, the robot (if going from the end state - the antitarget - to the start state - the robot's position) will have some end velocity. This velocity (and all velocities slower than it) satisfies the condition that from that given velocity, the robot can travel the determined distance and end at rest without accelerating above some threshold. This is important, because it lets us change at will the acceleration that the force field provides.
1. If this computed velocity is greater than the robot's max velocity, then the force field is "out of range" and the system uses the commanded velocity without
change.
1. Otherwise, the commanded velocity vector is split into two orthogonal components. One vector is pointing in the same direction as the robot's offset from the antitarget, and the other is perpendicular to that.
1. The perpendicular component of this vector has no impact on the robot's velocity *towards* the antitarget, so it will be unaffected throughout this process.
1. A "scale factor" is then determined that is used to quantify, between 0 and 1 inclusive, how much of the robot's max speed is being used to drive towards the antitarget. For example, a scale of 0 means that the drive is not requesting any change in distance between the robot and the antitarget at this instant, i.e. the driver is driving perfectly perpendicular to the offset vector. A scale of 1 indicates that the driver is asking for maximum speed in the direction of the offset vector.
1. That scale factor is used to scale up, or down, the maximum velocity (towards the antitarget) vector.
1. Finally, the scaled velocity vector and the original perpendicular-to-motion vectors are combined to create a new output that will slow down the robot's movement in the direction of travel towards the antitarget, but leavese the other component of the translation intact.

## In detail, with math
Please first have an understanding of basic vectors, the dot product, and basic vector operations.

### Step 1: Calculate the offset vector $x$

To find the position offset vector, the antitarget's position vector is subtracted from the robot's current position vector. This results in a vector $x$, which represents the positional offset between the robot and the antitarget. The magnitude of this vector, $|x|$, tells us the distance between the two positions.

### Step 2: Determine max velocity
This step only deals with the component of the robot's velocity parallel to the offset vector $x$. To determine end velocity, the following equation is used:

$$ v^2 = u^2 + 2as $$

Where $v$ represents the final velocity, $u$ is the initial velocity, $a$ is the acceleration, and $s$ is the distance.

Because the robot ends at rest, we know that the end velocity is $0$. We also know that the distance is $|x|$, and the acceleration is a constant that we can fine-tune. Because the $v^2$ term is $0$, solving for $u$ yields:

$$ u = \sqrt{-2as} $$

This makes sense. Because our acceleration is going to slow us down, it will be negative. Multiplying a negative acceleration by $-2$ and then by a positive distance gives a positive number in the square root.

We have arrived a new vector $t$, which is the vector that represents the maximum velocity in the direction of $x$. The perpendicular direction does not matter.

### Step 3: Split $c$ into its component vectors.
The input velocity vector $c$ can be split into two orthogonal components: perpendicular (which we will call $u$) with $t$ and parallel with $t$ (call it $i$).

Now let us construct a triangle. One side length will be made by the unmodified vector $c$, and the other will be made by $i$. So, this is $c$, and one of its components $i$. Taking the dot product of their unit vectors gives the cosine of the angle between the two vectors. That is,

$$ cos(\theta) = {i \cdot c \over |i| \cdot |c|} $$

However, recall that $i$ is collinear with $t$, which is known. So, their unit vectors are the same. This gives the following equation:

$$ cos(\theta) = {t \cdot c \over |t| \cdot |c|} $$

Now, both of these vectors are either known or already computed. This means that we can now try to solve for the actual vector $i$.

If we construct a triangle between the vectors $i$ and $u$, a triangle is constructed with legs of $u$ and $i$, and hypotenuse $c$.

The cosine of $\theta$ is the adjacent leg divided by the hypotenuse, so

$$ cos(\theta) = {|i| \over |c|} $$

Combining these last two equations:

$$ {|i| \over |c|} = {t \cdot c \over |t| \cdot |c|} $$

Typically, we would have to assume that $|c|$ is zero, but we can say that we don't really care, because then the dot product of $c$ and $t$ would be zero. So,

$$ |i| = {t \cdot c \over |t|} = \hat t \cdot c$$

Because $i$ is a scaled version of $t$, we can express $i$ in terms of $t$ and their magnitudes.

$$ i = t \cdot {|i| \over |t|} = \hat t \cdot |i| $$

Now, substituting in for $|i|$,

$$ i = \hat t \cdot (\hat t \cdot c) $$

Now, we can also solve for $u$ rather simply: $c = u + i$, so $u = c - i$.

### Step 4: Determine scale factor for $i$
We're approaching the end here. Let's remind ourselves of what we're trying to do.

After all that we've done, it boils down to two pretty basic steps: Firstly, we scale down $t$ based on the relative size of $i$ to create $a$. Then, we add $a$ and $u$ to create a final vector, $r$, which is the final robot velocity to apply.

To find this scale factor, consider what its bounds should be. A value of 1 means that $t$ is used, so the velocity towards the antitarget is as high as our program will allow. This would correspond to maximum value for $i$. But what is the maximum value of $i$? That's just the robot's maximum speed in that direction. However, because the robot's max speed in one direction is not separate from its speed in another, the following equation is revealed:

$$ |c| = \sqrt{|u|^2+|i|^2} $$

Holding $|u|$ constant, and solving for $max(|i|)$, we get:

$$ max(|i|) = \sqrt{[max(|c|)]^2-|u|^2} $$

The maximum value for $|c|$ is the robot's max speed, $V_{max}$

Now, a scale factor $f$ can be created as follows:

$$ f = {|i| \over max(|i|)} = {|i| \over \sqrt{V_{max}^2-|u|^2}} $$

Because $|u|$ is given by $|c - i|$, we can replace it with that.

$$ f = {|i| \over max(|i|)} = {|i| \over \sqrt{V_{max}^2-|x - i|^2}} $$


### Step 5: Combine $a$ and $u$

Because $a = f \cdot t$, we can re-write this as:

$$ a = t \cdot {|i| \over \sqrt{V_{max}^2-|x-i|^2}} $$

To find the final vector $r$,

$$ r = a + u = c - i + a$$

## Finer points

When computing $|t|$, if the value for it is more than the robot's max velocity, then the original input should be allowed, because the robot is too far away to worry about decceleration. The vector $c$ is the output for these situations.

If the dot product of $c$ and $t$ is negative, they are in opposite directions, and therefore the robot should not be limited in speed. Just use $c$ as the output.

If it ever happens that $x$ is at $(0, 0)$, then you have reached what is effectively a singularity and the program should probably just allow $c$ to be used as output to avoid any divide-by-zero errors.

It can be helpful to calculate the $x$ for a point between the antitarget and the robot's position, to allow for a radius around the target instead of a single point in 2d space. Just make sure to set $|a|$ to $0$ if inside this circle.