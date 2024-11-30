---
title: "Actions"
weight: 5 
---

# Actions

Actions help you define simple behaviors that are easy to combine into large
routines. By breaking down autonomous programs you make them easier to
understandand and modify. And then the base actions can be reused in new
autonomous programs so you never need to start from scratch. But most
importantly of all your code will play nice with the Road Runner Quickstart.
Let's see how this all works!

{{< hint info >}}
Actions are very similar to commands as implemented in libraries like 
[WPILib](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
and 
[FTCLib](https://docs.ftclib.org/ftclib/command-base/command-system). Road
Runner uses a different name for this pattern to distinguish its particular
design from these peer libraries. The ideas have also been explored
extensively outside the FIRST realm. Check out cooperative multitasking and
coroutines if you're interested.
{{< /hint >}}

## Overview

Each subsystem of a robot has certain basic behaviors. For a drivetrain,
these may include following a trajectory, moving toward a point, and turning in place;
for a shooter you may have spinning up, firing a ball, and loading from the
magazine. These are the smallest units of action that accomplish something
meaningful. But they range under hood from simply setting a motor power to
tracking a smooth path with a sophisticated controller. 

In code we'll represent the subsystems with classes and the actions with methods
that return `Action`:

<!-- sample: actionsDecl -->

Now to run an action, just call `runBlocking()`:

<!-- sample: actionsMoveToPoint -->

Despite the name "blocking," this method can still be interrupted by pressing
the stop button. This feature comes for free with actions and is much more
reliable than carefully checking for interruption at every phase in your op
mode. 

{{< hint warning >}}
But actions are no panacea for programming folly. It's perfectly possible to
write custom actions cannot be interrupted, usually with a misplaced
`Thread.sleep()` or `while` loop. Take care in composing your own actions and
interrupt your op modes regularly to catch any issues in advance of competition. 
{{< /hint >}}

Then with basic actions in place, you can combine them together into a complex
action. Sequential actions run a list of actions one at a time in order, while
parallel actions run a list of actions simultaneously until each has finished. 

Here's a rudimentary routine that executes the following steps:
1. Turn 90 degrees in place.
1. While following `shootingTraj`, spin up the shooter and fire a ball.

<!-- sample: actionsCompositeAction -->

That's all there is to it!

## Built-in Actions

The quickstart comes with a small set of actions to start from.
* `SleepAction`: sleep for a duration
* `SequentialAction`: execute a bunch of actions one after the other
* `ParallelAction`: execute a bunch of actions at the same time
* `FollowTrajectoryAction`: follow a trajectory (separate tank and mecanum versions)
* `TurnAction`: turn in place (separate tank and mecanum versions)

## Custom Actions

At their core, actions are long-running segments of code that execute in many
little steps. This property allows us to run two actions A and B in parallel
without using multiple threads. By executing "step A", "step B", "step A", ...
in alternating fashion, actions A and B appear to proceed concurrently. But the
illusion is easily ruined if "step A" takes a long time and starves B of the
chance to run.

To create a custom action, make a class that implements `Action` and with the
following two methods: 
* `public boolean run(TelemetryPacket packet)`: Code to run repeatedly while the
  method returns `true`. Any data added to `packet` will be sent to FTC
  Dashboard&mdash;see its [telemetry
  documentation](https://acmerobotics.github.io/ftc-dashboard/features#telemetry)
  for details. 

Calls to `run()` should complete quickly. Delays longer than 100ms will begin to
noticeably impinge on other actions. 

Let's look at a simple shooter spin-up action.

<!-- sample: actionsShooterComplete -->

Checking the velocity and adding it to telemetry doesn't take much time even
though the shooter may need seconds to reach the right speed. 

Also if you only use the class `SpinUp` inside `spinUp()` you can move the class
inside the method.

<!-- sample: actionsShooterRefactor -->
