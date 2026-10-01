---
title: MeepMeep Visualizer
---

## MeepMeep Visualizer

When working with Road Runner, teams may find it helpful to visualize paths
virtually with MeepMeep *prior* to running on a real robot. In particular, it
shows any points where continuity is broken and the robot must come to a
momentary stop. And for 1.0.x, MeepMeep catches tangent-related exceptions on
the `lineToX()` and `lineToY()` methods (and their variants) without needing to
rerun and repush to the robot when testing fixes.

<!-- It can also display velocity, acceleration and position data (as needed) to debug against telemetry from real-life robot runs. -->

To install and use, follow the instructions on the pages below:

* [1.0.x MeepMeep](https://github.com/acmerobotics/MeepMeep)
* [0.5.x MeepMeep](https://github.com/NoahBres/MeepMeep)

Choose the version that matches your robot code. Examples using
`TrajectorySequenceBuilder` and `addDisplacementMarker()` use the 0.5.x API.
Road Runner 1.0.x uses `TrajectoryActionBuilder` and actions instead; see the
[migration guide](../v1-0/migration/).

### Relocalization

MeepMeep displays the planned trajectory with ideal poses. It does not simulate
encoder error, AprilTag measurements, or a feedback controller correcting the
robot's estimated position. Changing the displayed robot pose does not change
the trajectory, and playback can overwrite that pose on the next update.

On a real robot, an AprilTag observation must first be converted to a robot pose
in the same field coordinate frame and units as Road Runner, accounting for the
camera's position and orientation on the robot. Apply accepted measurements to
the pose estimate used by your drive's localizer. The controller can then use
the corrected estimate while following the existing trajectory; correcting the
estimate does not rebuild or move the planned path.

If the next path should start at a corrected pose, finish the current action,
update the estimate, and then build the next action from that estimate at run
time. Building both paths beforehand captures the old starting pose. Use your
quickstart version's pose accessors, and test localization changes on the robot.
See the [quickstart discussion](https://github.com/acmerobotics/road-runner-quickstart/issues/367)
for integration considerations, including measurement latency and filtering.

A more detailed overview of MeepMeep features is given in [Learn Road
Runner](https://learnroadrunner.com/tools.html#meepmeep). 
