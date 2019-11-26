package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.path.PathBuilder
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator

/**
 * Easy-to-use builder for creating [Trajectory] instances.
 *
 * @param startPose start pose
 * @param trajectory initial trajectory (for splicing)
 * @param t time index in previous trajectory to begin new trajectory
 */
abstract class BaseTrajectoryBuilder protected constructor(
    startPose: Pose2d?,
    trajectory: Trajectory?,
    t: Double?,
    reversed: Boolean
) {
    private var pathBuilder: PathBuilder = if (startPose == null) {
        PathBuilder(trajectory!!.path, trajectory.profile[t!!].x, reversed)
    } else {
        PathBuilder(startPose, reversed)
    }

    private var temporalMarkers = mutableListOf<TemporalMarker>()
    private var spatialMarkers = mutableListOf<SpatialMarker>()

    /**
     * Adds a line path segment.
     *
     * @param pos end position
     * @param interpolator heading interpolator
     */
    @JvmOverloads
    fun lineTo(
        pos: Vector2d,
        interpolator: HeadingInterpolator = TangentInterpolator()
    ): BaseTrajectoryBuilder {
        pathBuilder.lineTo(pos, interpolator)

        return this
    }

    /**
     * Adds a strafe path segment.
     *
     * @param pos end position
     */
    fun strafeTo(pos: Vector2d): BaseTrajectoryBuilder {
        pathBuilder.strafeTo(pos)

        return this
    }

    /**
     * Adds a line straight forward.
     *
     * @param distance distance to travel forward
     */
    fun forward(distance: Double): BaseTrajectoryBuilder {
        pathBuilder.forward(distance)

        return this
    }

    /**
     * Adds a line straight backward.
     *
     * @param distance distance to travel backward
     */
    fun back(distance: Double): BaseTrajectoryBuilder {
        pathBuilder.back(distance)

        return this
    }

    /**
     * Adds a segment that strafes left in the robot reference frame.
     *
     * @param distance distance to strafe left
     */
    fun strafeLeft(distance: Double): BaseTrajectoryBuilder {
        pathBuilder.strafeLeft(distance)

        return this
    }

    /**
     * Adds a segment that strafes right in the robot reference frame.
     *
     * @param distance distance to strafe right
     */
    fun strafeRight(distance: Double): BaseTrajectoryBuilder {
        pathBuilder.strafeRight(distance)

        return this
    }

    /**
     * Adds a spline segment.
     *
     * @param pose end pose
     * @param interpolator heading interpolator
     */
    @Deprecated("raw heading interpolators are no longer permitted in high-level builders")
    fun splineTo(
        pose: Pose2d,
        interpolator: HeadingInterpolator = TangentInterpolator()
    ): BaseTrajectoryBuilder {
        pathBuilder.splineTo(pose, interpolator)

        return this
    }

    /**
     * Adds a spline segment with tangent heading interpolation.
     *
     * @param end end end
     */
    fun splineTo(end: Pose2d): BaseTrajectoryBuilder {
        pathBuilder.splineTo(end)

        return this
    }

    /**
     * Adds a spline segment with constant heading interpolation.
     *
     * @param end end end
     */
    fun splineToConstantHeading(end: Pose2d): BaseTrajectoryBuilder {
        pathBuilder.splineToConstantHeading(end)

        return this
    }

    /**
     * Adds a spline segment with linear heading interpolation.
     *
     * @param end end end
     */
    fun splineToLinearHeading(end: Pose2d, heading: Double): BaseTrajectoryBuilder {
        pathBuilder.splineToLinearHeading(end, heading)

        return this
    }

    /**
     * Adds a spline segment with spline heading interpolation.
     *
     * @param end end end
     */
    fun splineToSplineHeading(end: Pose2d, heading: Double): BaseTrajectoryBuilder {
        pathBuilder.splineToSplineHeading(end, heading)

        return this
    }

    /**
     * Adds a marker to the trajectory at [time].
     */
    fun addMarker(time: Double, callback: () -> Unit): BaseTrajectoryBuilder {
        temporalMarkers.add(TemporalMarker(time, callback))

        return this
    }

    /**
     * Adds a marker that will be triggered at the closest trajectory point to [point].
     */
    fun addMarker(point: Vector2d, callback: () -> Unit): BaseTrajectoryBuilder {
        spatialMarkers.add(SpatialMarker(point, callback))

        return this
    }

    /**
     * Adds a marker at the current position of the trajectory.
     */
    fun addMarker(callback: () -> Unit) =
        addMarker((pathBuilder.currentPose ?: pathBuilder.path!![pathBuilder.s!!]).vec(), callback)

    /**
     * Constructs the [Trajectory] instance.
     */
    fun build() = buildTrajectory(pathBuilder.build(), temporalMarkers, spatialMarkers)

    /**
     * Build a trajectory from [path].
     */
    protected abstract fun buildTrajectory(
        path: Path,
        temporalMarkers: List<TemporalMarker>,
        spatialMarkers: List<SpatialMarker>
    ): Trajectory
}
