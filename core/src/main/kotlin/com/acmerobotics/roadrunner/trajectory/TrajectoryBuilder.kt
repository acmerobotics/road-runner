package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints

private fun zeroPosition(state: MotionState) = MotionState(0.0, state.v, state.a, state.j)

/**
 * Builder for trajectories with *dynamic* constraints.
 */
class TrajectoryBuilder private constructor(
    startPose: Pose2d?,
    trajectory: Trajectory?,
    t: Double?,
    private val trajectoryConstraints: TrajectoryConstraints,
    private val start: MotionState,
    private val resolution: Double
) : BaseTrajectoryBuilder(startPose, trajectory, t) {
    /**
     * Create a builder from a start pose and motion state. This is the recommended constructor for creating
     * trajectories from rest.
     */
    constructor(
        startPose: Pose2d,
        trajectoryConstraints: TrajectoryConstraints,
        start: MotionState = MotionState(0.0, 0.0, 0.0),
        resolution: Double = 0.25
    ) : this(startPose, null, null, trajectoryConstraints, start, resolution)

    /**
     * Create a builder from an active trajectory. This is useful for interrupting a live trajectory and smoothly
     * transitioning to a new one.
     */
    constructor(
        trajectory: Trajectory,
        t: Double,
        trajectoryConstraints: TrajectoryConstraints,
        resolution: Double = 0.25
    ) : this(null, trajectory, t, trajectoryConstraints, zeroPosition(trajectory.profile[t]), resolution)

    override fun buildTrajectory(path: Path): Trajectory {
        val goal = MotionState(path.length(), 0.0, 0.0)
        return TrajectoryGenerator.generateTrajectory(path, trajectoryConstraints, start, goal, resolution)
    }
}
