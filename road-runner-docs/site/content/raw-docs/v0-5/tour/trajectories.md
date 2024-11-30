---
title: "Trajectories"
weight: 7
---

# Trajectories

Now it's time to combine paths with the motion profiles from earlier. Road Runner calls this composition a trajectory. Trajectories take time values and output the corresponding field frame kinematic state (i.e., real positions, velocities, and acceleration). This state can be transformed to the robot frame and fed directly into the feedforward component of the controller.

Here's a sample for planning a `Trajectory` from a `Path`:

{{< tabs "vel constraints" >}}
{{< tab "Java" >}}
```java
TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
    new TranslationalVelocityConstraint(20),
    new AngularVelocityConstraint(1)
));
TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(40);
Trajectory traj = TrajectoryGenerator.generateTrajectory(path, velConstraint, accelConstraint);
```
{{< /tab >}}

{{< tab "Kotlin" >}}
```kotlin
val velConstraint = MinVelocityConstraint(listOf(
    TranslationalVelocityConstraint(20.0),
    AngularVelocityConstraint(1.0)
))
val accelConstraint = ProfileAccelerationConstraint(40.0)
val traj = TrajectoryGenerator.generateTrajectory(path, velConstraint, accelConstraint)
```
{{< /tab >}}
{{</ tabs >}}

As indicated by their names, the constraints limit translational velocity, angular velocity, and profile acceleration. It's common to replace the translational velocity constraint with a stronger constraint that limits wheel velocities. These constraints are named according to the drive types (e.g., `MecanumVelocityConstraint`).

{{< hint info >}}
There is also a `TrajectoryBuilder` class that replicates the API of `PathBuilder` with a few additions.
{{</ hint >}}

Once a trajectory is finally generated, one of the `TrajectoryFollowers` can be used to generate the actual `DriveSignals` that are sent to the `Drive` class. The PIDVA followers are usually suitable, although `RamseteFollower` has noticeably better performance for tank drives.

Following a trajectory is as simple as creating a follower, calling `TrajectoryFollower.followTrajectory()`, and repeatedly querying `TrajectoryFollower.update()`:

{{< tabs "pidva follower" >}}
{{< tab "Java" >}}
```java
PIDCoefficients translationalPid = new PIDCoefficients(5, 0, 0);
PIDCoefficients headingPid = new PIDCoefficients(2, 0, 0);
HolonomicPIDVAFollower follower = new HolonomicPIDVAFollower(translationalPid, translationalPid, headingPid);

follower.followTrajectory(traj);

// call in loop
DriveSignal signal = follower.update(poseEstimate);
```
{{< /tab >}}

{{< tab "Kotlin" >}}
```kotlin
val translationalPid = PIDCoefficients(5.0, 0.0, 0.0)
val headingPid = PIDCoefficients(2.0, 0.0, 0.0)
val follower = HolonomicPIDVAFollower(translationalPid, translationalPid, headingPid)

follower.followTrajectory(traj)

// call in loop
val signal = follower.update(poseEstimate)
```
{{< /tab >}}
{{</ tabs >}}

