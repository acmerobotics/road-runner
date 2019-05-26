package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator
import com.acmerobotics.roadrunner.util.DoubleProgression
import com.acmerobotics.roadrunner.util.minus

/**
 * Path segment composed of a parametric curve and heading interpolator.
 *
 * @param curve parametric curve
 * @param interpolator heading interpolator
 * @param reversed if true, the segment is interpolated in reverse
 */
class PathSegment @JvmOverloads constructor(
    val curve: ParametricCurve,
    val interpolator: HeadingInterpolator = TangentInterpolator(),
    val reversed: Boolean = false
) {
    init {
        interpolator.init(curve)
    }

    fun length() = curve.length()

    @JvmOverloads
    operator fun get(s: Double, t: Double = reparam(s)): Pose2d {
        val point = if (reversed) {
            curve[curve.length() - s, t]
        } else {
            curve[s, t]
        }
        val heading = if (reversed) {
            interpolator[curve.length() - s, t]
        } else {
            interpolator[s, t]
        }
        return Pose2d(point.x, point.y, heading)
    }

    @JvmOverloads
    fun deriv(s: Double, t: Double = reparam(s)): Pose2d {
        val deriv = if (reversed) {
            -curve.deriv(curve.length() - s, t)
        } else {
            curve.deriv(s, t)
        }
        val headingDeriv = if (reversed) {
            -interpolator.deriv(curve.length() - s, t)
        } else {
            interpolator.deriv(s, t)
        }
        return Pose2d(deriv.x, deriv.y, headingDeriv)
    }

    @JvmOverloads
    fun secondDeriv(s: Double, t: Double = reparam(s)): Pose2d {
        val secondDeriv = if (reversed) {
            curve.secondDeriv(curve.length() - s, t)
        } else {
            curve.secondDeriv(s, t)
        }
        val headingSecondDeriv = if (reversed) {
            interpolator.secondDeriv(curve.length() - s, t)
        } else {
            interpolator.secondDeriv(s, t)
        }
        return Pose2d(secondDeriv.x, secondDeriv.y, headingSecondDeriv)
    }

    fun reparam(s: Double): Double {
        return if (reversed) {
            curve.reparam(curve.length() - s)
        } else {
            curve.reparam(s)
        }
    }

    internal fun reparam(s: DoubleProgression): DoubleArray {
        return if (reversed) {
            curve.reparam(curve.length() - s)
        } else {
            curve.reparam(s)
        }
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
