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
- [ ] Nice DSL/builder
- [ ] Various spline followers (PID, pure pursuit, time-varying non-linear feedback, gvf, etc.)
- [ ] Feedforward/drivetrain parameter tuning routines
- [ ] Spline optimizer
- [ ] Localization routines?

## GUI
- [x] Path view
- [x] Waypoint list
- [x] Basic constraint specification
- [ ] Heading control (for holonomic)
- [ ] Advanced constraint support
- [ ] Interactive waypoint dragging
- [ ] Curvature/profile visualization

## Plugin
- [ ] Path serialization/loading
- [ ] Path group management
- [ ] Live positional feedback