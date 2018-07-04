package com.acmerobotics.library.path.parametric

import com.acmerobotics.library.Pose2d
import com.acmerobotics.library.Vector2d
import com.acmerobotics.library.Waypoint
import com.acmerobotics.library.path.heading.HeadingInterpolator
import com.acmerobotics.library.path.heading.TangentInterpolator
import com.acmerobotics.library.util.InterpolatingTreeMap
import java.lang.Math.pow
import kotlin.math.sqrt

class QuinticSplineSegment(start: Waypoint, end: Waypoint, private val interpolator: HeadingInterpolator = TangentInterpolator()) {
    private val x: QuinticPolynomial = QuinticPolynomial(start.x, start.dx, start.dx2, end.x, end.dx, end.dx2)
    private val y: QuinticPolynomial = QuinticPolynomial(start.y, start.dy, start.dy2, end.y, end.dy, end.dy2)

    private val length: Double

    private val arcLengthSamples = InterpolatingTreeMap()

    companion object {
        private const val LENGTH_SAMPLES = 1000
    }

    init {
        arcLengthSamples[0.0] = 0.0
        val dx = 1.0 / LENGTH_SAMPLES
        var sum = 0.0
        var lastIntegrand = 0.0
        for (i in 1..LENGTH_SAMPLES) {
            val t = i * dx
            val deriv = internalDeriv(t)
            val integrand = sqrt(deriv.x * deriv.x + deriv.y * deriv.y) * dx
            sum += (integrand + lastIntegrand) / 2.0
            lastIntegrand = integrand

            arcLengthSamples[sum] = t
        }
        length = sum

        interpolator.fit(this)
    }

    private fun internalGet(t: Double) = Vector2d(x[t], y[t])

    private fun internalDeriv(t: Double) = Vector2d(x.deriv(t), y.deriv(t))

    private fun internalSecondDeriv(t: Double) = Vector2d(x.secondDeriv(t), y.secondDeriv(t))

    private fun internalThirdDeriv(t: Double) = Vector2d(x.thirdDeriv(t), y.thirdDeriv(t))

    fun displacementToParameter(displacement: Double) = arcLengthSamples.getInterpolated(displacement) ?: 0.0

    fun parameterDeriv(t: Double): Double {
        val deriv = internalDeriv(t)
        return 1.0 / sqrt(deriv.x * deriv.x + deriv.y * deriv.y)
    }

    fun parameterSecondDeriv(t: Double): Double {
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val numerator = -(deriv.x * secondDeriv.x + deriv.y * secondDeriv.y)
        val denominator = deriv.x * deriv.x + deriv.y * deriv.y
        return numerator / (denominator * denominator)
    }

    fun parameterThirdDeriv(t: Double): Double {
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val thirdDeriv = internalThirdDeriv(t)
        val firstNumerator = -(deriv.x * secondDeriv.x + deriv.y * secondDeriv.y)
        val secondNumeratorFirstTerm = secondDeriv.x * secondDeriv.x + secondDeriv.y * secondDeriv.y +
                deriv.x * thirdDeriv.x + deriv.y * thirdDeriv.y
        val secondNumeratorSecondTerm = -4.0 * firstNumerator
        val denominator = deriv.x * deriv.x + deriv.y * deriv.y
        return (secondNumeratorFirstTerm / pow(denominator, 2.5) +
                secondNumeratorSecondTerm / pow(denominator, 3.5))
    }

    fun length() = length

    operator fun get(displacement: Double) = internalGet(displacementToParameter(displacement))

    fun deriv(displacement: Double): Vector2d {
        val t = displacementToParameter(displacement)
        return internalDeriv(t) * parameterDeriv(t)
    }

    fun secondDeriv(displacement: Double): Vector2d {
        val t = displacementToParameter(displacement)
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val splineParameterDeriv = parameterDeriv(t)
        val splineParameterSecondDeriv = parameterSecondDeriv(t)
        return secondDeriv * splineParameterDeriv * splineParameterDeriv +
                deriv * splineParameterSecondDeriv
    }

    fun thirdDeriv(displacement: Double): Vector2d {
        val t = displacementToParameter(displacement)
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val thirdDeriv = internalThirdDeriv(t)
        val splineParameterDeriv = parameterDeriv(t)
        val splineParameterSecondDeriv = parameterSecondDeriv(t)
        val splineParameterThirdDeriv = parameterThirdDeriv(t)
        return thirdDeriv * splineParameterDeriv * splineParameterDeriv * splineParameterDeriv +
                secondDeriv * splineParameterSecondDeriv * splineParameterDeriv * 3.0 +
                deriv * splineParameterThirdDeriv
    }

    fun pose(displacement: Double): Pose2d {
        val point = get(displacement)
        val heading = interpolator[displacement]
        return Pose2d(point.x, point.y, heading)
    }

    fun poseDeriv(displacement: Double): Pose2d {
        val deriv = deriv(displacement)
        val headindDeriv = interpolator.deriv(displacement)
        return Pose2d(deriv.x, deriv.y, headindDeriv)
    }

    fun poseSecondDeriv(displacement: Double): Pose2d {
        val secondDeriv = secondDeriv(displacement)
        val headingSecondDeriv = interpolator.secondDeriv(displacement)
        return Pose2d(secondDeriv.x, secondDeriv.y, headingSecondDeriv)
    }

    fun endPose() = pose(length())

    fun endPoseDeriv() = poseDeriv(length())

    fun endPoseSecondDeriv() = poseSecondDeriv(length())

    override fun toString() = "(${x.a}*t^5+${x.b}*t^4+${x.c}*t^3+${x.d}*t^2+${x.e}*t+${x.f},${y.a}*t^5+${y.b}*t^4+${y.c}*t^3+${y.d}*t^2+${y.e}*t+${y.f})"
}