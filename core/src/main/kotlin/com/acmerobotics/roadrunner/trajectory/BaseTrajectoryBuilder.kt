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
    startHeading: Double?,
    trajectory: Trajectory?,
    t: Double?
) {
    private var pathBuilder: PathBuilder = if (startPose == null) {
        PathBuilder(trajectory!!.path, trajectory.profile[t!!].x)
    } else {
        PathBuilder(startPose, startHeading!!)
    }

    val currentPose
        get() = pathBuilder.currentPose
    val currentHeading
        get() = pathBuilder.currentHeading

    private var temporalMarkers = mutableListOf<RelativeTemporalMarker>()
    private var spatialMarkers = mutableListOf<SpatialMarker>()

    /**
     * Adds a line path segment.
     *
     * @param pos end position
     * @param interpolator heading interpolator
     */
    @Deprecated("raw heading interpolators are no longer permitted in high-level builders")
    fun lineTo(
        pos: Vector2d,
        interpolator: HeadingInterpolator = TangentInterpolator()
    ): BaseTrajectoryBuilder {
        pathBuilder.lineTo(pos, interpolator)

        return this
    }

    /**
     * Adds a line segment with tangent heading interpolation.
     *
     * @param position end position
     */
    fun lineTo(position: Vector2d): BaseTrajectoryBuilder {
        pathBuilder.lineTo(position)

        return this
    }

    /**
     * Adds a line segment with constant heading interpolation.
     *
     * @param position end position
     */
    fun lineToConstantHeading(position: Vector2d): BaseTrajectoryBuilder {
        pathBuilder.lineToConstantHeading(position)

        return this
    }

    /**
     * Adds a line segment with linear heading interpolation.
     *
     * @param position end position
     */
    fun lineToLinearHeading(position: Vector2d, heading: Double): BaseTrajectoryBuilder {
        pathBuilder.lineToLinearHeading(position, heading)

        return this
    }

    /**
     * Adds a line segment with spline heading interpolation.
     *
     * @param position end position
     */
    fun lineToSplineHeading(position: Vector2d, heading: Double): BaseTrajectoryBuilder {
        pathBuilder.lineToSplineHeading(position, heading)

        return this
    }


    /**
     * Adds a strafe path segment.
     *
     * @param position end position
     */
    fun strafeTo(position: Vector2d): BaseTrajectoryBuilder {
        pathBuilder.strafeTo(position)

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
     * @param pose end pose
     */
    fun splineTo(pose: Pose2d): BaseTrajectoryBuilder {
        pathBuilder.splineTo(pose)

        return this
    }

    /**
     * Adds a spline segment with constant heading interpolation.
     *
     * @param pose end pose
     */
    fun splineToConstantHeading(pose: Pose2d): BaseTrajectoryBuilder {
        pathBuilder.splineToConstantHeading(pose)

        return this
    }

    /**
     * Adds a spline segment with linear heading interpolation.
     *
     * @param pose end pose
     */
    fun splineToLinearHeading(pose: Pose2d, heading: Double): BaseTrajectoryBuilder {
        pathBuilder.splineToLinearHeading(pose, heading)

        return this
    }

    /**
     * Adds a spline segment with spline heading interpolation.
     *
     * @param pose end pose
     */
    fun splineToSplineHeading(pose: Pose2d, heading: Double): BaseTrajectoryBuilder {
        pathBuilder.splineToSplineHeading(pose, heading)

        return this
    }

    /**
     * Adds a marker to the trajectory at [time].
     */
    fun addMarker(time: Double, callback: MarkerCallback) = addMarker(0.0, time, callback)

    fun addMarker(scale: Double, offset: Double, callback: MarkerCallback) = addMarker({ scale * it + offset }, callback)

    fun addMarker(time: (Double) -> Double, callback: MarkerCallback): BaseTrajectoryBuilder {
        temporalMarkers.add(RelativeTemporalMarker(time, callback))

        return this
    }

    /**
     * Adds a marker that will be triggered at the closest trajectory point to [point].
     */
    fun addMarker(point: Vector2d, callback: MarkerCallback): BaseTrajectoryBuilder {
        spatialMarkers.add(SpatialMarker(point, callback))

        return this
    }

    /**
     * Adds a marker at the current position of the trajectory.
     */
    fun addMarker(callback: MarkerCallback) =
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
        temporalMarkers: List<RelativeTemporalMarker>,
        spatialMarkers: List<SpatialMarker>
    ): Trajectory
}
