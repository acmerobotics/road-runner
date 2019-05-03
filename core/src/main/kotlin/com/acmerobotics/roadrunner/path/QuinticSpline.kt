package com.acmerobotics.roadrunner.path

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.DoubleProgression
import kotlin.math.sqrt

private const val LENGTH_SAMPLES = 1000

/**
 * Combination of two quintic polynomials into a 2D quintic spline. See
 * [this short paper](https://github.com/acmerobotics/road-runner/blob/master/doc/pdf/Quintic_Splines_for_FTC.pdf) for
 * some motivation and implementation details.
 *
 * @param start start waypoint
 * @param end end waypoint
 */
class QuinticSpline(start: Waypoint, end: Waypoint) : ParametricCurve() {

    /**
     * X polynomial (i.e., x(t))
     */
    val x: QuinticPolynomial = QuinticPolynomial(start.x, start.dx, start.d2x, end.x, end.dx, end.d2x)

    /**
     * Y polynomial (i.e., y(t))
     */
    val y: QuinticPolynomial = QuinticPolynomial(start.y, start.dy, start.d2y, end.y, end.dy, end.d2y)

    /**
     * Class for representing the end points of interpolated quintic splines.
     *
     * @param x x position
     * @param y y position
     * @param dx x derivative
     * @param dy y derivative
     * @param d2x x second derivative
     * @param d2y y second derivative
     */
    class Waypoint @JvmOverloads constructor(
        val x: Double,
        val y: Double,
        val dx: Double = 0.0,
        val dy: Double = 0.0,
        val d2x: Double = 0.0,
        val d2y: Double = 0.0
    ) {
        fun pos() = Vector2d(x, y)

        fun deriv() = Vector2d(dx, dy)

        fun secondDeriv() = Vector2d(d2x, d2y)
    }

    private val length: Double

    private val sSamples = DoubleArray(LENGTH_SAMPLES + 1)
    private val tSamples = DoubleArray(LENGTH_SAMPLES + 1)

    init {
        sSamples[0] = 0.0
        tSamples[0] = 0.0

        val dx = 1.0 / LENGTH_SAMPLES
        var sum = 0.0
        var lastIntegrand = 0.0
        for (i in 1..LENGTH_SAMPLES) {
            val t = i * dx
            val deriv = internalDeriv(t)
            val integrand = sqrt(deriv.x * deriv.x + deriv.y * deriv.y) * dx
            sum += (integrand + lastIntegrand) / 2.0
            lastIntegrand = integrand

            sSamples[i] = sum
            tSamples[i] = t
        }
        length = sum
    }

    override fun internalGet(t: Double) = Vector2d(x[t], y[t])

    override fun internalDeriv(t: Double) = Vector2d(x.deriv(t), y.deriv(t))

    override fun internalSecondDeriv(t: Double) =
        Vector2d(x.secondDeriv(t), y.secondDeriv(t))

    override fun internalThirdDeriv(t: Double) =
        Vector2d(x.thirdDeriv(t), y.thirdDeriv(t))

    override fun reparam(s: Double): Double {
        if (s <= 0.0) return 0.0
        if (s >= length) return 1.0

        var lo = 0
        var hi = LENGTH_SAMPLES

        while (lo <= hi) {
            val mid = (hi + lo) / 2

            if (s < sSamples[mid]) {
                hi = mid - 1
            } else if (s > sSamples[mid]) {
                lo = mid + 1
            }
        }

        return tSamples[lo] + (s - sSamples[lo]) * (tSamples[hi] - tSamples[lo]) / (sSamples[hi] - sSamples[lo])
    }

    override fun reparam(s: DoubleProgression): DoubleArray {
        val t = DoubleArray(s.items())
        var i = 0
        var sampleIndex = 0
        var currS = s.start
        while (currS <= s.end) {
            t[i++] = when {
                currS <= 0.0 -> 0.0
                currS >= length -> 1.0
                else -> {
                    while (sSamples[sampleIndex] < currS) {
                        sampleIndex++
                    }
                    val s0 = sSamples[sampleIndex - 1]
                    val s1 = sSamples[sampleIndex]
                    val t0 = tSamples[sampleIndex - 1]
                    val t1 = tSamples[sampleIndex]
                    (t0 + (currS - s0) * (t1 - t0) / (s1 - s0))
                }
            }
            currS += s.step
        }
        while (i < t.size) {
            t[i] = 1.0
            i++
        }
        return t
    }

    override fun paramDeriv(t: Double): Double {
        val deriv = internalDeriv(t)
        return 1.0 / sqrt(deriv.x * deriv.x + deriv.y * deriv.y)
    }

    override fun paramSecondDeriv(t: Double): Double {
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val numerator = -(deriv.x * secondDeriv.x + deriv.y * secondDeriv.y)
        val denominator = deriv.x * deriv.x + deriv.y * deriv.y
        return numerator / (denominator * denominator)
    }

    override fun paramThirdDeriv(t: Double): Double {
        val deriv = internalDeriv(t)
        val secondDeriv = internalSecondDeriv(t)
        val thirdDeriv = internalThirdDeriv(t)
        val firstNumerator = -(deriv.x * secondDeriv.x + deriv.y * secondDeriv.y)
        val secondNumeratorFirstTerm = secondDeriv.x * secondDeriv.x + secondDeriv.y * secondDeriv.y +
                deriv.x * thirdDeriv.x + deriv.y * thirdDeriv.y
        val secondNumeratorSecondTerm = -4.0 * firstNumerator
        val denominator = deriv.x * deriv.x + deriv.y * deriv.y
        return (secondNumeratorFirstTerm / Math.pow(denominator, 2.5) +
                secondNumeratorSecondTerm / Math.pow(denominator, 3.5))
    }

    override fun length() = length

    override fun toString() = "($x,$y)"
}

operator fun Pair<Double, Double>.minus(other: Pair<Double, Double>) =
    Pair(first - other.first, second - other.second)
