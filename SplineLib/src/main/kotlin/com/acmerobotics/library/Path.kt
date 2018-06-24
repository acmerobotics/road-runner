package com.acmerobotics.library

import kotlin.math.abs

abstract class Path(private val headingInterpolator: HeadingInterpolator = TangentInterpolator()) {
    init {
        headingInterpolator.init(this) // TODO: this warning (and the general heading interpolation mechanism)
    }

    abstract fun length(): Double

    fun pose(displacement: Double): Pose2d {
        val pos = this[displacement]
        val heading = headingInterpolator[displacement]
        return Pose2d(pos.x, pos.y, heading)
    }

    fun poseDeriv(displacement: Double): Pose2d {
        val deriv = deriv(displacement)
        val headingDeriv = headingInterpolator.deriv(displacement)
        return Pose2d(deriv.x, deriv.y, headingDeriv)
    }

    fun poseSecondDeriv(displacement: Double): Pose2d {
        val secondDeriv = secondDeriv(displacement)
        val headingSecondDeriv = headingInterpolator.secondDeriv(displacement)
        return Pose2d(secondDeriv.x, secondDeriv.y, headingSecondDeriv)
    }

    abstract operator fun get(displacement: Double): Vector2d
    abstract fun deriv(displacement: Double): Vector2d
    abstract fun secondDeriv(displacement: Double): Vector2d
    abstract fun thirdDeriv(displacement: Double): Vector2d

    fun start() = get(0.0)
    fun end() = get(length())

    fun curvature(displacement: Double): Double {
        val deriv = deriv(displacement)
        val secondDeriv = secondDeriv(displacement)
        val norm = deriv.norm()
        return abs(deriv.x * secondDeriv.y - deriv.y * secondDeriv.x) / (norm * norm * norm)
    }
}