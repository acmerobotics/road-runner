package com.acmerobotics.library.path

import com.acmerobotics.library.Pose2d

class Path(private val parametricCurve: ParametricCurve, private val interpolator: HeadingInterpolator = TangentInterpolator()) {
    init {
        interpolator.init(parametricCurve)
    }

    fun length() = parametricCurve.length()

    operator fun get(displacement: Double): Pose2d {
        val point = parametricCurve[displacement]
        val heading = interpolator[displacement]
        return Pose2d(point.x, point.y, heading)
    }

    fun deriv(displacement: Double): Pose2d {
        val deriv = parametricCurve.deriv(displacement)
        val headindDeriv = interpolator.deriv(displacement)
        return Pose2d(deriv.x, deriv.y, headindDeriv)
    }

    fun secondDeriv(displacement: Double): Pose2d {
        val secondDeriv = parametricCurve.secondDeriv(displacement)
        val headingSecondDeriv = interpolator.secondDeriv(displacement)
        return Pose2d(secondDeriv.x, secondDeriv.y, headingSecondDeriv)
    }

    fun end() = get(length())

    fun endDeriv() = deriv(length())

    fun endSecondDeriv() = secondDeriv(length())
}