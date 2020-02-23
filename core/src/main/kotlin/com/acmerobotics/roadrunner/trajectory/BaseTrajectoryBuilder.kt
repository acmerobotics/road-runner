package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.path.PathBuilder

/**
 * Easy-to-use builder for creating [Trajectory] instances.
 *
 * @param startPose start pose
 * @param trajectory initial trajectory (for splicing)
 * @param t time index in previous trajectory to begin new trajectory
 */
@Suppress("UNCHECKED_CAST")
abstract class BaseTrajectoryBuilder<T : BaseTrajectoryBuilder<T>> protected constructor(
    startPose: Pose2d?,
    startHeading: Double?,
    trajectory: Trajectory?,
    t: Double?
) {
    protected var pathBuilder: PathBuilder = if (startPose == null) {
        PathBuilder(trajectory!!.path, trajectory.profile[t!!].x)
    } else {
        PathBuilder(startPose, startHeading!!)
    }

    private val temporalMarkers = mutableListOf<TemporalMarker>()
    private val displacementMarkers = mutableListOf<DisplacementMarker>()
    private val spatialMarkers = mutableListOf<SpatialMarker>()

    /**
     * Adds a line segment with tangent heading interpolation.
     *
     * @param endPosition end position
     */
    fun lineTo(endPosition: Vector2d): T {
        pathBuilder.lineTo(endPosition)

        return this as T
    }

    /**
     * Adds a line segment with constant heading interpolation.
     *
     * @param endPosition end position
     */
    fun lineToConstantHeading(endPosition: Vector2d): T {
        pathBuilder.lineToConstantHeading(endPosition)

        return this as T
    }

    /**
     * Adds a line segment with linear heading interpolation.
     *
     * @param endPosition end position
     * @param endHeading end heading
     */
    fun lineToLinearHeading(endPosition: Vector2d, endHeading: Double): T {
        pathBuilder.lineToLinearHeading(endPosition, endHeading)

        return this as T
    }

    /**
     * Adds a line segment with spline heading interpolation.
     *
     * @param endPosition end position
     */
    fun lineToSplineHeading(endPosition: Vector2d, endHeading: Double): T {
        pathBuilder.lineToSplineHeading(endPosition, endHeading)

        return this as T
    }

    /**
     * Adds a strafe path segment.
     *
     * @param endPosition end position
     */
    fun strafeTo(endPosition: Vector2d): T {
        pathBuilder.strafeTo(endPosition)

        return this as T
    }

    /**
     * Adds a line straight forward.
     *
     * @param distance distance to travel forward
     */
    fun forward(distance: Double): T {
        pathBuilder.forward(distance)

        return this as T
    }

    /**
     * Adds a line straight backward.
     *
     * @param distance distance to travel backward
     */
    fun back(distance: Double): T {
        pathBuilder.back(distance)

        return this as T
    }

    /**
     * Adds a segment that strafes left in the robot reference frame.
     *
     * @param distance distance to strafe left
     */
    fun strafeLeft(distance: Double): T {
        pathBuilder.strafeLeft(distance)

        return this as T
    }

    /**
     * Adds a segment that strafes right in the robot reference frame.
     *
     * @param distance distance to strafe right
     */
    fun strafeRight(distance: Double): T {
        pathBuilder.strafeRight(distance)

        return this as T
    }

    /**
     * Adds a spline segment with tangent heading interpolation.
     *
     * @param endPose end pose
     */
    fun splineTo(endPose: Pose2d): T {
        pathBuilder.splineTo(endPose)

        return this as T
    }

    /**
     * Adds a spline segment with constant heading interpolation.
     *
     * @param endPose end pose
     */
    fun splineToConstantHeading(endPose: Pose2d): T {
        pathBuilder.splineToConstantHeading(endPose)

        return this as T
    }

    /**
     * Adds a spline segment with linear heading interpolation.
     *
     * @param endPose end pose
     */
    fun splineToLinearHeading(endPose: Pose2d, endHeading: Double): T {
        pathBuilder.splineToLinearHeading(endPose, endHeading)

        return this as T
    }

    /**
     * Adds a spline segment with spline heading interpolation.
     *
     * @param endPose end pose
     */
    fun splineToSplineHeading(endPose: Pose2d, endHeading: Double): T {
        pathBuilder.splineToSplineHeading(endPose, endHeading)

        return this as T
    }

    /**
     * Adds a marker to the trajectory at [time].
     */
    fun addTemporalMarker(time: Double, callback: MarkerCallback) =
        addTemporalMarker(0.0, time, callback)

    /**
     * Adds a marker to the trajectory at [scale] * trajectory duration + [offset].
     */
    fun addTemporalMarker(scale: Double, offset: Double, callback: MarkerCallback) =
        addTemporalMarker({ scale * it + offset }, callback)

    /**
     * Adds a marker to the trajectory at [time] evaluated with the trajectory duration.
     */
    fun addTemporalMarker(time: (Double) -> Double, callback: MarkerCallback): T {
        temporalMarkers.add(TemporalMarker(time, callback))

        return this as T
    }

    /**
     * Adds a marker that will be triggered at the closest trajectory point to [point].
     */
    fun addSpatialMarker(point: Vector2d, callback: MarkerCallback): T {
        spatialMarkers.add(SpatialMarker(point, callback))

        return this as T
    }

    /**
     * Adds a marker at the current position of the trajectory.
     */
    fun addDisplacementMarker(callback: MarkerCallback) =
        addDisplacementMarker(pathBuilder.build().length(), callback)

    /**
     * Adds a marker to the trajectory at [displacement].
     */
    fun addDisplacementMarker(displacement: Double, callback: MarkerCallback) =
        addDisplacementMarker(0.0, displacement, callback)

    /**
     * Adds a marker to the trajectory at [scale] * path length + [offset].
     */
    fun addDisplacementMarker(scale: Double, offset: Double, callback: MarkerCallback) =
        addDisplacementMarker({ scale * it + offset }, callback)

    /**
     * Adds a marker to the trajectory at [displacement] evaluated with path length.
     */
    fun addDisplacementMarker(displacement: (Double) -> Double, callback: MarkerCallback): T {
        displacementMarkers.add(DisplacementMarker(displacement, callback))

        return this as T
    }

    /**
     * Constructs the [Trajectory] instance.
     */
    fun build() = buildTrajectory(pathBuilder.build(), temporalMarkers, displacementMarkers, spatialMarkers)

    /**
     * Build a trajectory from [path].
     */
    protected abstract fun buildTrajectory(
        path: Path,
        temporalMarkers: List<TemporalMarker>,
        displacementMarkers: List<DisplacementMarker>,
        spatialMarkers: List<SpatialMarker>
    ): Trajectory
}
