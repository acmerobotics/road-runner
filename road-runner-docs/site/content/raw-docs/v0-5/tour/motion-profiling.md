---
title: "Motion Profiling"
weight: 4
---

# Motion Profiling

For a moment, let's return to the elevator example. While the addition of a gravity feedforward in the last section improves the position control, the PID controller is still doing the majority of the work to follow step responses. When a new position is commanded, the error immediately spikes and the controller saturates, sending the carriage at the mechanism's maximum acceleration. Soon the carriage slows down and overshoots the setpoint a little before settling around the commanded position. While efficient, the sudden acceleration causes unnecessary mechanical/electrical strain and the resultant overshoot wastes time. For drivetrain movements, there are additional issues with wheel slippage.

The most common solution to this problem is simple: put a low cap on actuator speed. This ad-hoc solution only partially addresses the issue at hand and cripples the robot. A better solution is to consider the full kinematic constraints of the system. Instead of just limiting maximum velocity, one should also limit maximum acceleration and maximum jerk \(jerk is the derivative of acceleration\). Before the motion begins, the controller pre-plans a "motion profile" that describes the robot's position, velocity, etc. over time.

![](/sample-jerk-limited-profile.png "Jerk-limited 60-inch motion profile (vmax = 25 in/s, amax = 40 in/s^2, jmax = 100 in/s^3)")

Instead of instantly trying to reach the setpoint, the PID controller now tracks the profile. The heaving lifting is now done by the motion profile instead of the PID loop.

The profile above was generated using the following code:

{{< tabs "simple profile" >}}
{{< tab "Java" >}}
```java
MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
    new MotionState(0, 0, 0),
    new MotionState(60, 0, 0),
    25,
    40,
    100
);
```
{{< /tab >}}

{{< tab "Kotlin" >}}
```kotlin
val profile = MotionProfileGenerator.generateSimpleMotionProfile(
    MotionState(0.0, 0.0, 0.0),
    MotionState(60.0, 0.0, 0.0),
    25.0,
    40.0,
    100.0
)
```
{{< /tab >}}
{{</ tabs >}}

To follow the profile, simply feed the velocity and acceleration for the corresponding time to the `PIDFController` at each timestep:

{{< tabs "profile get" >}}
{{< tab "Java" >}}
```java
MotionState state = profile.get(elapsedTime);

controller.setTargetPosition(state.x);
controller.setTargetVelocity(state.v);
controller.setTargetAcceleration(state.a);

double correction = controller.update(measuredPosition);
```
{{< /tab >}}

{{< tab "Kotlin" >}}
```kotlin
val state = profile[elapsedTime]

controller.apply {
    targetPosition = state.x
    targetVelocity = state.v
    targetAcceleration = state.a
}

val correction = controller.update(measuredPosition)
```
{{< /tab >}}
{{</ tabs >}}



