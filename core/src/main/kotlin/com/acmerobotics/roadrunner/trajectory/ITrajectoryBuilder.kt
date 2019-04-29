package com.acmerobotics.roadrunner.trajectory

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints

interface ITrajectoryBuilder {
    fun reverse(): ITrajectoryBuilder
    fun setReversed(reversed: Boolean): ITrajectoryBuilder
    fun turn(angle: Double, constraintsOverride: DriveConstraints? = null): ITrajectoryBuilder
    fun turnTo(heading: Double, constraintsOverride: DriveConstraints? = null): ITrajectoryBuilder
    fun face(pos: Vector2d): ITrajectoryBuilder
    fun lineTo(pos: Vector2d, interpolator: HeadingInterpolator = TangentInterpolator(), constraintsOverride: TrajectoryConstraints? = null): ITrajectoryBuilder
    fun strafeTo(pos: Vector2d): ITrajectoryBuilder
    fun forward(distance: Double): ITrajectoryBuilder
    fun back(distance: Double): ITrajectoryBuilder
    fun strafeLeft(distance: Double): ITrajectoryBuilder
    fun strafeRight(distance: Double): ITrajectoryBuilder
    fun splineTo(pose: Pose2d, interpolator: HeadingInterpolator = TangentInterpolator(), constraintsOverride: TrajectoryConstraints? = null): ITrajectoryBuilder
    fun waitFor(duration: Double): ITrajectoryBuilder
    fun beginComposite(): ITrajectoryBuilder
    fun closeComposite(): ITrajectoryBuilder
    fun build(): Trajectory
}