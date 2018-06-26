package com.acmerobotics.library.path

import com.acmerobotics.library.trajectory.PathMotionConstraints
import com.acmerobotics.library.Pose2d

class HolonomicPath(
    private val path: Path,
    private val headingInterpolator: HeadingInterpolator = TangentInterpolator()
) {
    val motionConstraints: PathMotionConstraints = path.motionConstraints

    init {
        headingInterpolator.init(path)
    }

    fun length() = path.length()

    fun start() = get(0.0)
    fun end() = get(length())

    fun startDeriv() = deriv(0.0)
    fun endDeriv() = deriv(length())

    fun startSecondDeriv() = secondDeriv(0.0)
    fun endSecondDeriv() = secondDeriv(length())

    operator fun get(displacement: Double): Pose2d {
        val pos = path[displacement]
        val heading = headingInterpolator[displacement]
        return Pose2d(pos.x, pos.y, heading)
    }

    fun deriv(displacement: Double): Pose2d {
        val pos = path.deriv(displacement)
        val heading = headingInterpolator.deriv(displacement)
        return Pose2d(pos.x, pos.y, heading)
    }

    fun secondDeriv(displacement: Double): Pose2d {
        val pos = path.secondDeriv(displacement)
        val heading = headingInterpolator.secondDeriv(displacement)
        return Pose2d(pos.x, pos.y, heading)
    }
}