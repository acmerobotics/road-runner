package com.acmerobotics.splinelib.path

import com.acmerobotics.splinelib.Pose2d
import com.acmerobotics.splinelib.path.heading.HeadingInterpolator
import com.acmerobotics.splinelib.path.heading.TangentInterpolator

class Path(val parametricCurve: ParametricCurve, val interpolator: HeadingInterpolator = TangentInterpolator(), val reversed: Boolean = false) {
    init {
        interpolator.init(parametricCurve)
    }

    fun length() = parametricCurve.length()

    operator fun get(displacement: Double): Pose2d {
        val point = if (reversed) {
            parametricCurve[length() - displacement]
        } else {
            parametricCurve[displacement]
        }
        val heading = if (reversed) {
            interpolator[length() - displacement]
        } else {
            interpolator[displacement]
        }
        return Pose2d(point.x, point.y, heading)
    }

    fun deriv(displacement: Double): Pose2d {
        val deriv = if (reversed) {
            -parametricCurve.deriv(length() - displacement)
        } else {
            parametricCurve.deriv(displacement)
        }
        val headingDeriv = if (reversed) {
            -interpolator.deriv(length() - displacement)
        } else {
            interpolator.deriv(displacement)
        }
        return Pose2d(deriv.x, deriv.y, headingDeriv)
    }

    fun secondDeriv(displacement: Double): Pose2d {
        val secondDeriv = if (reversed) {
            parametricCurve.secondDeriv(length() - displacement)
        } else {
            parametricCurve.secondDeriv(displacement)
        }
        val headingSecondDeriv = if (reversed) {
            interpolator.secondDeriv(length() - displacement)
        } else {
            interpolator.secondDeriv(displacement)
        }
        return Pose2d(secondDeriv.x, secondDeriv.y, headingSecondDeriv)
    }

    fun start() = get(0.0)

    fun startDeriv() = deriv(0.0)

    fun startSecondDeriv() = secondDeriv(0.0)

    fun end() = get(length())

    fun endDeriv() = deriv(length())

    fun endSecondDeriv() = secondDeriv(length())
}