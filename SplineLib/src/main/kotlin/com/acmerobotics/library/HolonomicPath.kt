package com.acmerobotics.library

class HolonomicPath(
    private val path: Path,
    private val headingInterpolator: HeadingInterpolator = TangentInterpolator(path)
) {

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

    fun curvature(displacement: Double) = path.curvature(displacement)
}