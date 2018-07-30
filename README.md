motion-planner
==============
A simple Kotlin library for planning 2D mobile robot paths and trajectories designed for FTC.

# Features

## Core
- [x] Quintic ~~bezier~~ splines
- [x] Point turns
- [x] Linear segments
- [x] Dynamic constraint-capable trapezoidal motion profiling
- [x] Heading interpolators
- [x] ~~Modifiers for common FTC drivetrains~~
- [x] Nice ~~DSL~~/builder
- [ ] First class dashboard and Java support
- [ ] PIDV trajectory follower
- [ ] Feedforward/drivetrain parameter tuning routines
- [ ] Comprehensive quickstart and examples
- [ ] Docs
- [ ] Ramsete trajectory follower
- [ ] PP path follower?
- [ ] GVF path follower?

## GUI
- [x] Path view
- [x] Waypoint list
- [x] Basic constraint specification
- [x] Curvature/profile visualization
- [ ] Virtual file system updates
- [ ] Advanced constraint support
- [ ] Interactive waypoint dragging
- [ ] Heading control (for holonomic)

## Plugin
- [x] Path serialization/loading
- [ ] Path group management
- [ ] Live positional feedback