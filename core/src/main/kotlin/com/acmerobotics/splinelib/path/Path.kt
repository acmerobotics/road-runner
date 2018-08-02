package com.acmerobotics.splinelib.path

import com.acmerobotics.splinelib.Pose2d
import com.acmerobotics.splinelib.path.heading.HeadingInterpolator
import com.acmerobotics.splinelib.path.heading.TangentInterpolator

/**
 * Path composed of a parametric curve and a heading interpolator.
 *
 * @param parametricCurve parametric curve
 * @param interpolator heading interpolator
 * @param reversed whether or not to travel along the path in reverse
 */
class Path @JvmOverloads constructor(
        val parametricCurve: ParametricCurve,
        val interpolator: HeadingInterpolator = TangentInterpolator(),
        val reversed: Boolean = false
) {
    init {
        interpolator.init(parametricCurve)
    }

    /**
     * Returns the length of the path.
     */
    fun length() = parametricCurve.length()

    /**
     * Returns the pose [displacement] units along the path.
     */
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

    /**
     * Returns the pose derivative [displacement] units along the path.
     */
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

    /**
     * Returns the pose second derivative [displacement] units along the path.
     */
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

    /**
     * Returns the start pose.
     */
    fun start() = get(0.0)

    /**
     * Returns the start pose derivative.
     */
    fun startDeriv() = deriv(0.0)

    /**
     * Returns the start pose second derivative.
     */
    fun startSecondDeriv() = secondDeriv(0.0)

    /**
     * Returns the end pose.
     */
    fun end() = get(length())

    /**
     * Returns the end pose derivative.
     */
    fun endDeriv() = deriv(length())

    /**
     * Returns the end pose second derivative.
     */
    fun endSecondDeriv() = secondDeriv(length())
}