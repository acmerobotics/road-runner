# Cancellation

Sometimes you may want to a stop a trajectory when it's already running. For
example, if you're executing an intake trajectory in autonomous, you may want to
stop once sensors detect that a game element was taken in.

Cancellation comes in two forms, abrupt and smooth. The former has the advantage
of speed over the latter, though rapid acceleration may cause wheel slip and
impede following performance.

## Abrupt Cancellation

Abrupt cancellation is easy to implement. Simply switch from executing a
trajectory to stopping the motors or executing a new trajectory. 

<!-- sample: actionsCancelAbruptly -->

Then this cancelable action should be executed concurrently with logic that
decides when to cancel. We'll take inspiration from [teleop
actions](../teleop-actions/) and write our own `runBlocking()` loop.

<!-- sample: cancelableActionOpMode -->

It's a little silly to call `cancelAbruptly()` when we could just `break;`
directly here, but this pattern can be used in more complex scenarios where 
the cancelable action can also be nested inside another action. In this case, 
when the trajectory action is cancelled, it won't poison the rest of the sequential
action. The sequence will skip forward to the next action (sleep in this case)
and continue like normal.

<!-- sample: cancelableActionOpMode -->

Nothing in the implementation really depends on `action` being a
`FollowTrajectoryAction`. We can generalize the pattern into a `FailoverAction`
that runs a main action until instructed to fall back to a secondary action.

<!-- sample: actionsFailover -->

The cancellable trajectory action from above can now be represented as

<!-- sample: actionsFailoverExample -->

You can also put `NullAction` in the failover slot to do nothing on
`failover()`.

## Smooth Cancellation

Smooth cancellation can be implemented with a failover action that switches to a
stopping trajectory on cancellation. Stopping trajectories can be generated from
any normal `Trajectory` by calling `cancel()` with how far the robot is along
the trajectory. 

{{< hint info >}}
Will add more details here in the future. See
[road-runner-quickstart#310](https://github.com/acmerobotics/road-runner-quickstart/issues/310)
in the meantime.
{{< /hint >}}

<!-- What's required for this to go well? Maybe `trajectoryBuilder` in mec drive -->

<!-- TODO

I'll preface this by distinguishing abrupt cancellation from smooth cancellation. Abrupt cancellation entails immediately stopping the current action and commanding a new one. Teams may be familiar with this from LRR's interrupting a live trajectory section. Smooth cancellation involves coming to a stop as soon as possible while still respecting the constraints of the trajectory. Cancellation as mentioned in "new features" and elsewhere in RR core refers to smooth cancellation, and the rest of this comment outlines how to achieve that.

Cancellation is supported by the core library through the Trajectory type (not CancelableTrajectory as the docs still say...), though it isn't exposed by TrajectoryActionBuilder or in the quickstart. (I may extend TrajectoryActionBuilder to support cancellation natively by producing a CancellableAction or something under the hood.)

To get a Trajectory, you can use TrajectoryBuilder (build() gives you a sequence of them). You can call cancel() on that object to get a DisplacementTrajectory that eases the robot to a stop. One can imagine putting this together into an Action or something that can switch from following the main (base) trajectory and failing over to a cancellation trajectory when commanded.

In general, the foundations are there for a sufficiently ambitious team. It's just not quite ready and mature enough for everyone else. -->
