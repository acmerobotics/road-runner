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
    t: Double?
) {
    private var pathBuilder: PathBuilder = if (startPose == null) {
        PathBuilder(trajectory!!.path, trajectory.profile[t!!].x)
    } else {
        PathBuilder(startPose)
    }

    private var temporalMarkers = mutableListOf<TemporalMarker>()
    private var spatialMarkers = mutableListOf<SpatialMarker>()

    /**
     * Reverse the direction of robot travel.
     */
    fun reverse(): BaseTrajectoryBuilder {
        pathBuilder.reverse()

        return this
    }

    /**
     * Sets the robot travel direction.
     */
    fun setReversed(reversed: Boolean): BaseTrajectoryBuilder {
        pathBuilder.setReversed(reversed)

        return this
    }

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
    @JvmOverloads
    fun splineTo(
        pose: Pose2d,
        interpolator: HeadingInterpolator = TangentInterpolator()
    ): BaseTrajectoryBuilder {
        pathBuilder.splineTo(pose, interpolator)

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
