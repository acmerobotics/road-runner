---
title: "Builder Reference"
weight: 100 
---

# Builder Reference

## Path Primitives

The begin pose is the origin with a heading of {{< katex >}} \frac{\pi}{6} {{< /katex >}}.

### `lineToX()`

```java
.lineToX(48)
```

{{< video lineToX >}}

### `lineToY()`

```java
.lineToY(36)
```

{{< video lineToY >}}

### `splineTo()`

```java
.splineTo(new Vector2d(48, 48), Math.PI / 2)
```

{{< video splineTo >}}

## Heading Primitives

The begin pose is the origin with a heading of {{< katex >}} \frac{\pi}{2} {{< /katex >}}.

### Tangent Heading (default)

```java
.setTangent(0)
.splineTo(new Vector2d(48, 48), Math.PI / 2)
```

{{< video tangentHeading >}}

### Constant Heading

```java
.setTangent(0)
.splineToConstantHeading(Vector2d(48, 48), Math.PI / 2)
```

{{< video constantHeading >}}

### Linear Heading

```java
.setTangent(0)
.splineToLinearHeading(Pose2d(48, 48, 0), Math.PI / 2)
```

{{< video linearHeading >}}

### Spline Heading

```java
.setTangent(0)
.splineToSplineHeading(Pose2d(48, 48, 0), Math.PI / 2)
```

{{< video splineHeading >}}

## Actions and Timing

### `afterTime()`

`afterTime(dt, action)` starts an action in parallel `dt` seconds after the end of
the preceding trajectory segment, turn, or sequential action. At the beginning
of a builder, the delay is measured from the start of the built action.

To start a mechanism at the same time as a movement, place the marker first:

```java
drive.actionBuilder(beginPose)
    .afterTime(0, slider.highBucket())
    .strafeTo(new Vector2d(40, 32))
    .build();
```

Placing the marker after `strafeTo()` starts the mechanism at the planned end of
that segment instead:

```java
drive.actionBuilder(beginPose)
    .strafeTo(new Vector2d(40, 32))
    .afterTime(0, slider.highBucket())
    .build();
```

Markers within a trajectory use its planned timing, not a sensor check that the
robot reached the endpoint. Later builder steps can run while the marker's
action is still active. To wait for both a movement and a mechanism before the
next step, use [a parallel group inside a sequence](../actions/#timing-and-waiting-for-parallel-actions).

### `waitSeconds()` and `stopAndAdd()`

`waitSeconds(t)` ends the pending trajectory and adds a wait of `t` seconds to
the sequence. Fractional values such as `0.9` are supported. `stopAndAdd(action)`
ends the pending trajectory and runs the supplied action before proceeding.

```java
drive.actionBuilder(beginPose)
    .strafeTo(new Vector2d(40, 32))
    .waitSeconds(0.9)
    .stopAndAdd(() -> intake.setPower(1.0))
    .waitSeconds(0.9)
    .stopAndAdd(() -> intake.setPower(0.0))
    .build();
```

This waits 0.9 seconds after the drive action finishes, then runs the intake for
0.9 seconds. A callback that sets motor power finishes immediately; it does not
wait for the motor to reach a position or stop. All callbacks must return
quickly so the scheduler can advance the other actions and timers.
