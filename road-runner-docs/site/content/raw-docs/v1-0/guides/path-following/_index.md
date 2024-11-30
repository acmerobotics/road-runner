# Path Following

Teams almost always use RR to generate and follow trajectories that specify
where the robot should be at every point in time. To follow a trajectory, the
robot controller starts a stopwatch and directs the feedback controller to close
the gap between the estimated pose and the desired pose at the current time from
the trajectory. A key advantage of this approach is the clear termination
criterion: when the stopwatch hits the trajectory duration.

The main disadvantage is that you can't make full use of your robot's
capabilities. The feedback controller needs some headroom to speed up if it
falls behind, and that headroom leaves performance on the table. And if the
robot gets ahead of its trajectory, the feedback controller will push it
backwards away from the goal which leaves time on the table.

If you want to push your robot to its limits, you can try a different following
paradigm that doesn't used a fixed temporal schedule. Instead of using a
stopwatch to compute a desired pose, the controller finds the closest point on
the path to the robot and moves based on it.

{{< hint info >}}
This approach is similar to pure pursuit which you may have heard of before. The
main difference is that our proposed path follower doesn't have a lookahead and
attempts higher-accuracy tracking of the path instead of merely pushing toward
the goal. 
{{< /hint >}}

To see how to implement this, let's work up from the `HolonomicController`
class. It implements one method with the signature

```kotlin
fun compute(
    targetPose: Pose2dDual<Time>,
    actualPose: Pose2d,
    actualVelActual: PoseVelocity2d,
): PoseVelocity2dDual<Time>
```

The latter two arguments come from the localizer as with trajectory. The
question is how to get `targetPose`, a pose on the path and its derivative with
respect to time. We can call `project(path, actualPose, displacementGuess)` to
get a displacement along `path`. (`project()` works iteratively, and you can
help it by passing the last displacement as a guess for the next loop
iteration.) Now the target pose is given by `path.get(displacement, 3)` where the 3
indicates that derivatives up to second order are desired. But these derivatives
aren't quite what `compute()` wants. They are with respect to the displacement
instead of time (i.e., we have a `Pose2dDual<Arclength>` instead of a
`Pose2dDual<Time>`). Following the types, there's a method on `Pose2dDual<T>` that
does what we want:

```kotlin
fun <NewParam> reparam(oldParam: DualNum<NewParam>) =
    Pose2dDual(position.reparam(oldParam), heading.reparam(oldParam))
```

Now we just need the displacement/arclength as a `DualNum<Time>`. One way to
compute that is evaluating a `DisplacementProfile` at the displacement value
from `project()`. It's not quite kinematically correct, though it works when
`actualPose` is close to `targetPose` (which should hopefully be the case). (A
more correct expression is left as an exercise to the reader. Hint: try taking
the derivative of the orthogonality constraint between the path
point/derivatives and the query point.)

Returning to the practical side, there are additional changes to truly push the
robot velocity is fast as possible. The velocity command calculation in
`HolonomicController` assumes that adding the feedforward command and feedback
command together will always be feasible (i.e., within physical voltage limits).
To guarantee this while maximizing the commanded speed of the motors, start by
computing the feedback 

```kotlin
val error = targetPose.value().minusExp(actualPose)

val feedbackCommand = PoseVelocity2d(
    Vector2d(
        axialPosGain * error.position.x,
        lateralPosGain * error.position.y,
    ),
    headingGain * error.heading.log(),
) +
PoseVelocity2d(
    Vector2d(
        axialVelGain * velErrorActual.linearVel.x,
        lateralVelGain * velErrorActual.linearVel.y,
    ),
    headingVelGain * velErrorActual.angVel,
)
```

Then take the raw `Pose2dDual<Arclength>` from before and compute the maximum
path displacement velocity/time derivative that satisfies the constraints (the
commanded voltage for any motor is no larger than the current battery voltage).
You probably don't care about predicting acceleration in this formulation and
should operate on the assumption that `kA` is zero.
