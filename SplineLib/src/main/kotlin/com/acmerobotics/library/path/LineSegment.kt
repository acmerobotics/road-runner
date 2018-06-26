package com.acmerobotics.library.path

import com.acmerobotics.library.Pose2d
import com.acmerobotics.library.Vector2d

class LineSegment(val start: Vector2d, val end: Vector2d) : Path() {
    companion object {
        fun fromPoses(start: Pose2d, end: Pose2d) =
            HolonomicPath(LineSegment(start.pos(), end.pos()), LinearInterpolator(start.heading, end.heading))
    }

    private val diff = end - start

    override fun length() = diff.norm()

    override fun get(displacement: Double) = start + diff * displacement / length()

    override fun deriv(displacement: Double) = diff / length()

    override fun secondDeriv(displacement: Double) = Vector2d(0.0, 0.0)

    override fun thirdDeriv(displacement: Double) = Vector2d(0.0, 0.0)
}