---
title: "New Features"
weight: 2
---

# New Features

The biggest change is a **new quickstart** redesigned from the ground up to give better
calibration results in less time. It also incorporates community wisdom around control
schemes and has first-class support for dead wheel odometry.

On top of that, there are **significant changes to the API of the core library**. The 
mathematical tools and approach remain largely the same, but they have a fresh coat of
paint. And then there a few notable features added to the library:

* **No more `PathContinuityViolationException` errors!** The usual builders now
  produce sequences of paths or trajectories. Instead of failing on a continuity
  violation, they split the path/trajectory. 

* **Motion profiles and constraints support asymmetric acceleration limits.**
  can now specify deceleration and acceleration limits on profile and
  trajectories. The constructor of `ProfileAccelerationConstraint` now accepts
  two values, a minimum (negative) acceleration and a maximum (positive)
  acceleration. 

* **Trajectories can be canceled smoothly.** A canceled trajectory brings
  the robot to a stop as soon as possible while remaining on the path and
  obeying the mandated constraints. See `Trajectory#cancel()`.

* **Unit-adjusted Ramsete in the quickstart by default.** The old tank PIDVA
  follower is no longer in either the quickstart or the core library. Ramsete is
  ubiquitous in FRC and performs much better than its predecessor. The
  [defaults from
  WPILib](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html)
  have been incorporated, so you may not even need to tune regardless of your
  units. 

<!--
* **Projection-based path following works out of the box.** Here's some starting boilerplate:
  
  ```java
  public void followTrajAsPath(Trajectory t) {
      DisplacementTrajectory dt = new DisplacementTrajectory(t);
      HolonomicController c = new HolonomicController(0, 0, 0);
      double disp = 0;
      while (disp + 1 < dt.length()) {
          Twist2d robotVelRobot = updatePoseEstimateAndGetActualVel();
          disp = dt.project(pose, disp);
          Pose2dDual<Time> poseTarget = dt.get(disp);
          Twist2dDual<Time> cmd = c.compute(poseTarget, pose, robotVelRobot);

          MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(cmd);
          double voltage = voltageSensor.getVoltage();
          leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
          leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
          rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
          rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);
      }

      leftFront.setPower(0);
      leftBack.setPower(0);
      rightBack.setPower(0);
      rightFront.setPower(0);
  }
  ```
-->
  