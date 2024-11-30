---
title: "Migrating from 0.5.x"
weight: 3
---

# Migrating from 0.5.x

This document assesses the 1.0.0 API changes that will most affect teams using the
library. 

## Packages

Previously, classes were scattered across many subpackages of
`com.acmerobotics.roadrunner`. Now everything can be found in the root package. 

## Trajectory Builder

Most of the methods are the same and should feel familiar. The most notable change 
is splitting `lineTo()` into two methods, `lineToX()` and `lineToY()`. Line directions
will now automatically match the path tangent for continuity unless `setTangent()` is
called. Wherever possible, the trajectory builder encourages continuity.

## Trajectory Sequences

<!-- (You can read more about actions on [this page](../actions).) -->

Trajectory sequences have been extended intoa a full actions system. Each action
describes a task that executes in a bunch of small steps. For example, following
a trajectory is an action made up of "read encoders, compute deviation from the
trajectory, set motor powers, read encoders, compute deviation ...". The
`TrajectoryActionBuilder` class takes the place of `TrajectorySequenceBuilder`
in making it easy to combine following trajectories with moving servos, setting
motor powers, and any other behaviors you might want.

Markers are included in the new system. Here's an example:

```java
builder
.lineToX(-12)
.afterDisp(5, dispMarker)
.splineTo(Vector2d(-24, -12), -Math.PI / 2)
.turn(Math.PI / 2)
.afterTime(0.5, tempMarker)
.turn(Math.PI / 2)
.setReversed(true)
.splineTo(Vector2d(-36, 0), Math.PI)
```

The action `dispMarker` triggers 5 units after the end of the first line and 
is similar to a displacement marker. The action `tempMarker` triggers half of a second
after the first turn begins and is similar to a temporal marker.

{{< video markers >}}

## PID Controller

Some old classes didn't make the cut for the new code. Among this group is `PIDFController`,
which you can find [here in Java](https://gist.github.com/rbrott/89a2b1ce538ec860851670faeb0b721e).
All of the other old files are still available [on GitHub](https://github.com/acmerobotics/road-runner/tree/423c9554c97ecac249b8d07eff43235496b22f39), and you can copy them into your codebase if you need to.
Nothing is gone completely.
