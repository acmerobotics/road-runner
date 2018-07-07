package com.acmerobotics.splinelib.path

import com.acmerobotics.splinelib.Pose2d

class Path(val parametricCurve: ParametricCurve, val interpolator: HeadingInterpolator = TangentInterpolator()) {
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

    fun start() = get(0.0)

    fun startDeriv() = deriv(0.0)

    fun startSecondDeriv() = secondDeriv(0.0)

    fun end() = get(length())

    fun endDeriv() = deriv(length())

    fun endSecondDeriv() = secondDeriv(length())
}