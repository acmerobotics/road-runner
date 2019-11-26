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
    reversed: Boolean,
    private val trajectoryConstraints: TrajectoryConstraints,
    private val start: MotionState,
    private val resolution: Double
) : BaseTrajectoryBuilder(startPose, trajectory, t, reversed) {
    /**
     * Create a builder from a start pose and motion state. This is the recommended constructor for creating
     * trajectories from rest.
     */
    @JvmOverloads constructor(
        startPose: Pose2d,
        trajectoryConstraints: TrajectoryConstraints,
        start: MotionState = MotionState(0.0, 0.0, 0.0),
        resolution: Double = 0.25,
        reversed: Boolean = false
    ) : this(startPose, null, null, reversed, trajectoryConstraints, start, resolution)

    /**
     * Create a builder from an active trajectory. This is useful for interrupting a live trajectory and smoothly
     * transitioning to a new one.
     */
    @JvmOverloads constructor(
        trajectory: Trajectory,
        t: Double,
        trajectoryConstraints: TrajectoryConstraints,
        resolution: Double = 0.25,
        reversed: Boolean = false
    ) : this(null, trajectory, t, reversed, trajectoryConstraints, zeroPosition(trajectory.profile[t]), resolution)

    override fun buildTrajectory(
        path: Path,
        temporalMarkers: List<TemporalMarker>,
        spatialMarkers: List<SpatialMarker>
    ): Trajectory {
        val goal = MotionState(path.length(), 0.0, 0.0)
        return TrajectoryGenerator.generateTrajectory(
            path,
            trajectoryConstraints,
            start,
            goal,
            temporalMarkers,
            spatialMarkers,
            resolution
        )
    }
}
