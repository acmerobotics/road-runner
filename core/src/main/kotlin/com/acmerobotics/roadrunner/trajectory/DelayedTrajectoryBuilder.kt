package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints

open class DelayedTrajectoryBuilder @JvmOverloads constructor(
        private val getStartPose: () -> Pose2d,
        private val globalConstraints: DriveConstraints,
        private val resolution: Int = 250
) : ITrajectoryBuilder {

    private val segments = ArrayList<(ITrajectoryBuilder) -> Unit>()

    override fun reverse(): ITrajectoryBuilder {
        segments.add { it.reverse() }
        return this
    }

    override fun setReversed(reversed: Boolean): ITrajectoryBuilder {
        segments.add { it.setReversed(reversed) }
        return this
    }

    override fun turn(angle: Double, constraintsOverride: DriveConstraints?): ITrajectoryBuilder {
        segments.add { it.turn(angle, constraintsOverride) }
        return this
    }

    override fun turnTo(heading: Double, constraintsOverride: DriveConstraints?): ITrajectoryBuilder {
        segments.add { it.turnTo(heading, constraintsOverride) }
        return this
    }

    override fun face(pos: Vector2d): ITrajectoryBuilder {
        segments.add { it.face(pos) }
        return this
    }

    override fun lineTo(pos: Vector2d, interpolator: HeadingInterpolator, constraintsOverride: TrajectoryConstraints?): ITrajectoryBuilder {
        segments.add { it.lineTo(pos, interpolator, constraintsOverride) }
        return this
    }

    override fun strafeTo(pos: Vector2d): ITrajectoryBuilder {
        segments.add { it.strafeTo(pos) }
        return this
    }

    override fun forward(distance: Double): ITrajectoryBuilder {
        segments.add { it.forward(distance) }
        return this
    }

    override fun back(distance: Double): ITrajectoryBuilder {
        segments.add { it.back(distance) }
        return this
    }

    override fun strafeLeft(distance: Double): ITrajectoryBuilder {
        segments.add { it.strafeLeft(distance) }
        return this
    }

    override fun splineTo(pose: Pose2d, interpolator: HeadingInterpolator, constraintsOverride: TrajectoryConstraints?): ITrajectoryBuilder {
        segments.add { it.splineTo(pose, interpolator, constraintsOverride) }
        return this
    }

    override fun strafeRight(distance: Double): ITrajectoryBuilder {
        segments.add { it.strafeRight(distance) }
        return this
    }

    override fun waitFor(duration: Double): ITrajectoryBuilder {
        segments.add { it.waitFor(duration) }
        return this
    }

    override fun beginComposite(): ITrajectoryBuilder {
        segments.add { it.beginComposite() }
        return this
    }

    override fun closeComposite(): ITrajectoryBuilder {
        segments.add { it.closeComposite() }
        return this
    }

    override fun build(): Trajectory {
        val builder = TrajectoryBuilder(getStartPose(), globalConstraints, resolution)
        segments.forEach { it(builder) }
        return builder.build()
    }
}