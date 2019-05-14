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
 * @param constraints global drive constraints (overridable for specific segments)
 * @param resolution resolution used for path-based segments (see [Trajectory])
 */
abstract class BaseTrajectoryBuilder(startPose: Pose2d) {
    private var pathBuilder = PathBuilder(startPose)

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
     * @param constraintsOverride spline-specific constraints
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
     * Constructs the [Trajectory] instance.
     */
    fun build() = buildTrajectory(pathBuilder.build())

    /**
     * Build a trajectory from [path].
     */
    protected abstract fun buildTrajectory(path: Path): Trajectory
}
