package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.profile.SimpleMotionConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints
import com.acmerobotics.roadrunner.util.Angle
import kotlin.math.PI

private fun zeroPosition(state: MotionState) = MotionState(0.0, state.v, state.a, state.j)

private data class ConstraintsInterval(
    val start: Double,
    val end: Double,
    val constraints: TrajectoryConstraints
)

private class MergedTrajectoryConstraints(
    val baseConstraints: TrajectoryConstraints,
    val constraintsOverrideIntervals: List<ConstraintsInterval>
) : TrajectoryConstraints {
    override fun get(s: Double, pose: Pose2d, deriv: Pose2d, secondDeriv: Pose2d): SimpleMotionConstraints {
        for (interval in constraintsOverrideIntervals) {
            val (start, end, constraints) = interval
            if (s in start..end) {
                return constraints[s, pose, deriv, secondDeriv]
            }
        }
        return baseConstraints[s, pose, deriv, secondDeriv]
    }
}

/**
 * Builder for trajectories with *dynamic* constraints.
 */
class TrajectoryBuilder private constructor(
    startPose: Pose2d?,
    startHeading: Double?,
    trajectory: Trajectory?,
    t: Double?,
    private val constraints: TrajectoryConstraints,
    private val start: MotionState,
    private val resolution: Double
) : BaseTrajectoryBuilder<TrajectoryBuilder>(startPose, startHeading, trajectory, t) {
    /**
     * Create a builder from a start pose and motion state. This is the recommended constructor for creating
     * trajectories from rest.
     */
    @JvmOverloads constructor(
        startPose: Pose2d,
        startHeading: Double = startPose.heading,
        constraints: TrajectoryConstraints,
        resolution: Double = 0.25
    ) : this(startPose, startHeading, null, null, constraints, MotionState(0.0, 0.0, 0.0), resolution)

    @JvmOverloads constructor(
        startPose: Pose2d,
        reversed: Boolean,
        constraints: TrajectoryConstraints,
        resolution: Double = 0.25
    ) : this(startPose, Angle.norm(startPose.heading + if (reversed) PI else 0.0), constraints, resolution)

    /**
     * Create a builder from an active trajectory. This is useful for interrupting a live trajectory and smoothly
     * transitioning to a new one.
     */
    @JvmOverloads constructor(
        trajectory: Trajectory,
        t: Double,
        constraints: TrajectoryConstraints,
        resolution: Double = 0.25
    ) : this(null, null, trajectory, t, constraints, zeroPosition(trajectory.profile[t]), resolution)

    private val constraintsOverrideIntervals = mutableListOf<ConstraintsInterval>()

    /**
     * Adds a line segment with tangent heading interpolation.
     *
     * @param endPosition end position
     * @param constraintsOverride segment-specific constraints
     */
    fun lineTo(endPosition: Vector2d, constraintsOverride: TrajectoryConstraints): TrajectoryBuilder {
        val start = pathBuilder.build().length()

        lineTo(endPosition)

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a line segment with constant heading interpolation.
     *
     * @param endPosition end position
     * @param constraintsOverride segment-specific constraints
     */
    fun lineToConstantHeading(endPosition: Vector2d, constraintsOverride: TrajectoryConstraints): TrajectoryBuilder {
        val start = pathBuilder.build().length()

        lineToConstantHeading(endPosition)

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a line segment with linear heading interpolation.
     *
     * @param endPosition end position
     * @param endHeading end heading
     * @param constraintsOverride segment-specific constraints
     */
    fun lineToLinearHeading(endPosition: Vector2d, endHeading: Double, constraintsOverride: TrajectoryConstraints): TrajectoryBuilder {
        val start = pathBuilder.build().length()

        lineToLinearHeading(endPosition, endHeading)

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a line segment with spline heading interpolation.
     *
     * @param endPosition end position
     * @param endHeading end heading
     * @param constraintsOverride segment-specific constraints
     */
    fun lineToSplineHeading(endPosition: Vector2d, endHeading: Double, constraintsOverride: TrajectoryConstraints): TrajectoryBuilder {
        val start = pathBuilder.build().length()

        lineToLinearHeading(endPosition, endHeading)

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a strafe path segment.
     *
     * @param endPosition end position
     * @param constraintsOverride segment-specific constraints
     */
    fun strafeTo(endPosition: Vector2d, constraintsOverride: TrajectoryConstraints): TrajectoryBuilder {
        val start = pathBuilder.build().length()

        strafeTo(endPosition)

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a line straight forward.
     *
     * @param distance distance to travel forward
     * @param constraintsOverride segment-specific constraints
     */
    fun forward(distance: Double, constraintsOverride: TrajectoryConstraints): TrajectoryBuilder {
        val start = pathBuilder.build().length()

        forward(distance)

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a line straight backward.
     *
     * @param distance distance to travel backward
     * @param constraintsOverride segment-specific constraints
     */
    fun back(distance: Double, constraintsOverride: TrajectoryConstraints) =
        forward(-distance, constraintsOverride)

    /**
     * Adds a segment that strafes left in the robot reference frame.
     *
     * @param distance distance to strafe left
     * @param constraintsOverride segment-specific constraints
     */
    fun strafeLeft(distance: Double, constraintsOverride: TrajectoryConstraints): TrajectoryBuilder {
        val start = pathBuilder.build().length()

        strafeLeft(distance)

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a segment that strafes right in the robot reference frame.
     *
     * @param distance distance to strafe right
     * @param constraintsOverride segment-specific constraints
     */
    fun strafeRight(distance: Double, constraintsOverride: TrajectoryConstraints) =
        strafeLeft(-distance, constraintsOverride)

    /**
     * Adds a spline segment with tangent heading interpolation.
     *
     * @param endPose end pose
     * @param constraintsOverride segment-specific constraints
     */
    fun splineTo(endPose: Pose2d, constraintsOverride: TrajectoryConstraints): TrajectoryBuilder {
        val start = pathBuilder.build().length()

        splineTo(endPose)

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a spline segment with constant heading interpolation.
     *
     * @param endPose end pose
     * @param constraintsOverride segment-specific constraints
     */
    fun splineToConstantHeading(endPose: Pose2d, constraintsOverride: TrajectoryConstraints): TrajectoryBuilder {
        val start = pathBuilder.build().length()

        splineToConstantHeading(endPose)

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a spline segment with linear heading interpolation.
     *
     * @param endPose end pose
     * @param constraintsOverride segment-specific constraints
     */
    fun splineToLinearHeading(endPose: Pose2d, endHeading: Double, constraintsOverride: TrajectoryConstraints): TrajectoryBuilder {
        val start = pathBuilder.build().length()

        splineToLinearHeading(endPose, endHeading)

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    /**
     * Adds a spline segment with spline heading interpolation.
     *
     * @param endPose end pose
     * @param constraintsOverride segment-specific constraints
     */
    fun splineToSplineHeading(endPose: Pose2d, endHeading: Double, constraintsOverride: TrajectoryConstraints): TrajectoryBuilder {
        val start = pathBuilder.build().length()

        splineToSplineHeading(endPose, endHeading)

        val end = pathBuilder.build().length()

        constraintsOverrideIntervals.add(ConstraintsInterval(start, end, constraintsOverride))

        return this
    }

    override fun buildTrajectory(
        path: Path,
        temporalMarkers: List<TemporalMarker>,
        displacementMarkers: List<DisplacementMarker>,
        spatialMarkers: List<SpatialMarker>
    ): Trajectory {
        val goal = MotionState(path.length(), 0.0, 0.0)
        return TrajectoryGenerator.generateTrajectory(
            path,
            MergedTrajectoryConstraints(constraints,
                mutableListOf<ConstraintsInterval>().apply { addAll(constraintsOverrideIntervals) }),
            start,
            goal,
            temporalMarkers,
            displacementMarkers,
            spatialMarkers,
            resolution
        )
    }
}
