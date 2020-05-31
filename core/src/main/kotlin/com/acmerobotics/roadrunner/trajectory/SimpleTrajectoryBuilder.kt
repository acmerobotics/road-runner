package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.util.Angle
import kotlin.math.PI

private fun zeroPosition(state: MotionState) = MotionState(0.0, state.v, state.a, state.j)

/**
 * Builder for trajectories with *static* constraints.
 */
class SimpleTrajectoryBuilder private constructor(
    startPose: Pose2d?,
    startTangent: Double?,
    trajectory: Trajectory?,
    t: Double?,
    private val driveConstraints: DriveConstraints,
    private val start: MotionState
) : BaseTrajectoryBuilder<SimpleTrajectoryBuilder>(startPose, startTangent, trajectory, t) {
    /**
     * Create a builder from a start pose and motion state. This is the recommended constructor for creating
     * trajectories from rest.
     */
    @JvmOverloads constructor(
        startPose: Pose2d,
        startTangent: Double = startPose.heading,
        driveConstraints: DriveConstraints
    ) : this(startPose, startTangent, null, null, driveConstraints, MotionState(0.0, 0.0, 0.0))

    constructor(
        startPose: Pose2d,
        reversed: Boolean,
        driveConstraints: DriveConstraints
    ) : this(startPose, Angle.norm(startPose.heading + if (reversed) PI else 0.0), driveConstraints)

    /**
     * Create a builder from an active trajectory. This is useful for interrupting a live trajectory and smoothly
     * transitioning to a new one.
     */
    constructor(
        trajectory: Trajectory,
        t: Double,
        driveConstraints: DriveConstraints
    ) : this(null, null, trajectory, t, driveConstraints, zeroPosition(trajectory.profile[t]))

    override fun buildTrajectory(
        path: Path,
        temporalMarkers: List<TemporalMarker>,
        displacementMarkers: List<DisplacementMarker>,
        spatialMarkers: List<SpatialMarker>
    ): Trajectory {
        val goal = MotionState(path.length(), 0.0, 0.0)
        return TrajectoryGenerator.generateSimpleTrajectory(
            path,
            driveConstraints,
            start,
            goal,
            temporalMarkers,
            displacementMarkers,
            spatialMarkers
        )
    }
}
