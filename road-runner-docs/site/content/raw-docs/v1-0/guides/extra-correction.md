# Extra Correction

Road Runner uses feedback control while following trajectories to correct for
disturbances and end up in the desired final pose. But sometimes this correction
isn't enough. If you're using a mecanum drive, you can try to continue running
the feedback controller to achieve further accuracy. (This kind of "pose
stabilization" control is not possible with tank drives. Turning can help
eliminate residual heading error, though correcting the position requires more
elaborate strategies that are beyond the scope of this guide.)

One way to accomplish this is to modify the `TrajectoryFollowerAction` to
continue running even after the end of the trajectory is reached. The normal
termination criterion is

```java
if (t >= timeTrajectory.duration) {
    leftFront.setPower(0);
    leftBack.setPower(0);
    rightBack.setPower(0);
    rightFront.setPower(0);

    return false;
}
```

Say you want to continue until the position error is less than 2 inches. First
move some code from below to be above the conditional.

```diff
@@ -261,6 +261,13 @@ public final class MecanumDrive {
                 t = Actions.now() - beginTs;
             }
 
+            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
+            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));
+
+            PoseVelocity2d robotVelRobot = updatePoseEstimate();
+
+            Pose2d error = txWorldTarget.value().minusExp(pose);
+
             if (t >= timeTrajectory.duration) {
                 leftFront.setPower(0);
                 leftBack.setPower(0);
@@ -270,11 +277,6 @@ public final class MecanumDrive {
                 return false;
             }
 
-            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
-            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));
-
-            PoseVelocity2d robotVelRobot = updatePoseEstimate();
-
             PoseVelocity2dDual<Time> command = new HolonomicController(
                     PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                     PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
@@ -304,7 +306,6 @@ public final class MecanumDrive {
             p.put("y", pose.position.y);
             p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
 
-            Pose2d error = txWorldTarget.value().minusExp(pose);
             p.put("xError", error.position.x);
             p.put("yError", error.position.y);
             p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));
```

Then you can modify the condition to read

```java
if (t >= timeTrajectory.duration && error.position.norm() < 2) {
    // ...
    return false;
}
```

You may also want to check that the velocity is below a certain threshold in
case the robot overshoots the desired position.

```java
if (t >= timeTrajectory.duration && error.position.norm() < 2
                    && robotVelRobot.linearVel.norm() < 0.5) {
    // ...
    return false;
}
```

You can also add similar checks on heading and angular velocity.

But beware infinite loops! If something goes awry and the new conditions are
never satisfied, the robot will stay in place for the rest of the autonomous
program. Here's the previous condition with a timeout of 1 second:

```java
if ((t >= timeTrajectory.duration && error.position.norm() < 2
                    && robotVelRobot.linearVel.norm() < 0.5)
                    || t >= timeTrajectory.duration + 1) {
    // ...
    return false;
}
```
