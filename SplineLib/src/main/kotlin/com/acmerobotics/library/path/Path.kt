package com.acmerobotics.library.path

import com.acmerobotics.library.Pose2d
import com.acmerobotics.library.path.heading.HeadingInterpolator
import com.acmerobotics.library.path.heading.TangentInterpolator
import com.acmerobotics.library.path.parametric.ParametricCurve

class Path(
    private val parametricCurve: ParametricCurve,
    private val headingInterpolator: HeadingInterpolator = TangentInterpolator(parametricCurve)
) {
    fun length() = parametricCurve.length()

    fun start() = get(0.0)
    fun end() = get(length())

    fun startDeriv() = deriv(0.0)
    fun endDeriv() = deriv(length())

    fun startSecondDeriv() = secondDeriv(0.0)
    fun endSecondDeriv() = secondDeriv(length())

    operator fun get(displacement: Double): Pose2d {
        val pos = parametricCurve[displacement]
        val heading = headingInterpolator[displacement]
        return Pose2d(pos.x, pos.y, heading)
    }

    fun deriv(displacement: Double): Pose2d {
        val pos = parametricCurve.deriv(displacement)
        val heading = headingInterpolator.deriv(displacement)
        return Pose2d(pos.x, pos.y, heading)
    }

    fun secondDeriv(displacement: Double): Pose2d {
        val pos = parametricCurve.secondDeriv(displacement)
        val heading = headingInterpolator.secondDeriv(displacement)
        return Pose2d(pos.x, pos.y, heading)
    }
}